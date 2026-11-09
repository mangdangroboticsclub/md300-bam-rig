/* driver_board.c  -  SPI backend for the BAM identification firmware.
 *
 * Trimmed port of the SPI servo driver of MangDang's robot ESP32 firmware.
 * The wire protocol is identical; left out are the 12-servo sync write,
 * save/restore and the board-variant channel remap, which the bridge does
 * not need.
 */
#include "driver_board.h"

#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us */

#define TAG "BAMDB"

/* ---- pin map: MangDang MP4 ESP32 CORE (from the board schematic) ----
 *
 * Schematic nets: IO6_SPI_MOSI, IO16_SPI_CLK, IO17_SPI_MISO,
 *                 IO15_SPI_CS1 (CN3), IO7_SPI_CS2 (CN4),
 *                 IO4_SPI_CS3  (CN5), IO5_SPI_CS4 (CN6).
 *
 * SPI2_HOST on purpose: ESP32-S3 GP-SPI2 has six chip-select lines, GP-SPI3
 * only three, and the servo bus needs four.
 *
 * There is NO servo power-enable pin on this board: the four HC-PHD-2 servo
 * connectors are fed straight from Vbat+ (3S Li, 12.6 V full / 12 V adapter)
 * through OR-ing diodes, so the servo rail is live whenever power is plugged
 * in. There is also no battery-voltage sense divider, so the rail cannot be
 * read in firmware.
 */
#define SPI_MASTER_ID    SPI2_HOST   /* six CS lines; SPI3 has three */
#define SPI_MASTER_MOSI  6
#define SPI_MASTER_MISO  17
#define SPI_MASTER_CLK   16

/* Chip selects in connector order: CN3, CN4, CN5, CN6. */
#define SPI_CS_CN3       15
#define SPI_CS_CN4       7
#define SPI_CS_CN5       4
#define SPI_CS_CN6       5

/* board index -> connector (as in the robot firmware):
 *   0 -> CN4 (servos 1-3)     1 -> CN5 (servos 4-6)
 *   2 -> CN3 (servos 7-9)     3 -> CN6 (servos 10-12)                  */
#define SPI_CS0  SPI_CS_CN4
#define SPI_CS1  SPI_CS_CN5
#define SPI_CS2  SPI_CS_CN3
#define SPI_CS3  SPI_CS_CN6

/* ---- on-the-wire protocol (identical to AT32 spi_command_frame) ---- */
#define START_FIELD  0xA5A5
#define MODE_FIELD   0x0001   /* AT32: position control, "torque" field = max current mA */
#define START_CONFIG 0xC0DE

#pragma pack(push,1)
typedef struct { uint16_t mode, position; int16_t torque; uint16_t kp, kd; } servo_cmd_sub_t;
typedef struct { uint16_t start, mode; servo_cmd_sub_t s1, s2, s3; uint16_t check_sum; } host_SMS_t;

typedef struct { uint16_t status, position; int16_t torque; uint32_t res; } servo_fb_sub_t;
typedef struct { uint16_t start, status; servo_fb_sub_t s1, s2, s3; uint16_t check_sum; } SMS_host_t;

/* config frame, padded to the same 36 bytes as host_SMS_t */
typedef struct {
    uint16_t start, op, servo_index, param_id;
    float    value;
    uint8_t  padding[24];
} cfg_frame_t;
#pragma pack(pop)
_Static_assert(sizeof(host_SMS_t) == 36, "host_SMS_t size");
_Static_assert(sizeof(cfg_frame_t) == 36, "cfg_frame_t size");

enum {
    CFG_OP_NOP      = 0,
    CFG_OP_SET      = 1,
    CFG_OP_GET      = 2,
    CFG_OP_SAVE     = 3,
    CFG_OP_RESTORE  = 4,
    CFG_OP_GET_LIVE = 5,
};

static spi_device_handle_t dev[4];

/* Cached feedback. `valid` is cleared whenever the servo's board fails to
 * answer, so callers can tell "no board" from a real 0 reading. */
static uint16_t fb_posdd[12];
static int16_t  fb_cur[12];
static uint32_t fb_res[12];      /* reserved word: reserved1 = NTC temperature (0.1 degC) */
static uint16_t fb_status[12];   /* board status word of the last feedback frame */
static bool     fb_valid[12];

bool driver_board_feedback_valid(uint16_t servo)
{
    if (servo < 1 || servo > 12) return false;
    return fb_valid[servo - 1];
}

/* shadow of the last commanded state per servo (deci-degrees), so a direct
 * single-servo write can resend the board frame without disturbing the
 * other two servos on the same board. Channels the host never addressed
 * stay IDLE: a module may have an output held in the rig's anchor, and
 * the first frame must not put it in position mode. */
static uint16_t sh_mode[12]  = { [0 ... 11] = DB_MODE_IDLE };
static uint16_t sh_posdd[12] = { [0 ... 11] = 1550 };   /* db_scale_dd/2 */
static int16_t  sh_cur[12]   = { 0 };

/* SCS full-scale in AT32 deci-degrees. MUST always equal
 * range_position_deg x 10 on the AT32 boards. */
static float db_scale_dd = 3100.0f;
void  driver_board_set_scale_dd(float v){ if(v > 100.0f && v <= 3600.0f) db_scale_dd = v; }
float driver_board_get_scale_dd(void){ return db_scale_dd; }

static esp_err_t spi_xfer(uint8_t board, uint8_t *tx, uint8_t *rx)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length    = (size_t)36 * 8;   /* all frames are 36 bytes */
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (board > 3) return ESP_FAIL;
    return spi_device_transmit(dev[board], &t);
}

void driver_board_init(void)
{
    /* No power-enable pin on this board -- see the pin-map note above. */

    spi_bus_config_t bus = {
        .mosi_io_num   = SPI_MASTER_MOSI,
        .miso_io_num   = SPI_MASTER_MISO,
        .sclk_io_num   = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
        /* Enable the internal pull-up on MISO. With no driver board plugged
         * into a chip select, MISO would otherwise FLOAT and read back as
         * random noise, so "no board" could not be told apart from garbage
         * data. Pulling it up makes an absent board read deterministically
         * all-ones (0xFFFF), which board_resend() detects. */
        .flags = SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_MASTER_ID, &bus, SPI_DMA_CH_AUTO));
    gpio_set_pull_mode(SPI_MASTER_MISO, GPIO_PULLUP_ONLY);

    /* 4 driver-board devices (12 MHz, mode 0) */
    spi_device_interface_config_t cfg = {
        .mode           = 0,
        .duty_cycle_pos = 128,
        .clock_speed_hz = 12 * 1000 * 1000,   /* 12 MHz (15 MHz max) */
        .queue_size     = 2,
    };
    int cs[4] = { SPI_CS0, SPI_CS1, SPI_CS2, SPI_CS3 };
    for (int i = 0; i < 4; i++) {
        cfg.spics_io_num = cs[i];
        ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &cfg, &dev[i]));
    }

    ESP_LOGI(TAG, "driver board SPI%d init OK (MOSI%d MISO%d CLK%d, "
                  "CS board0..3 = CN4/CN5/CN3/CN6, 12 servos)",
             SPI_MASTER_ID + 1, SPI_MASTER_MOSI, SPI_MASTER_MISO, SPI_MASTER_CLK);
}

/* Send one board's frame rebuilt from the shadow, refresh feedback cache.
 *
 * Returns false if the SPI transaction failed OR the board did not answer.
 *
 * DETECTION OF AN ABSENT BOARD
 * The AT32 does NOT echo the command header back on a normal command frame:
 * its reply has { start, status, <3x feedback>, check_sum } where `start` and
 * `status` are the AT32's own values, not START_FIELD. (Only a *config* reply
 * is identifiable by listening for START_CONFIG in `status` - see
 * cfg_request().) So the reply cannot be validated by looking for a magic
 * number.
 *
 * What DOES distinguish a live board from an empty chip select is the
 * checksum. A present AT32 mirrors the command's own 16-bit check_sum field
 * back in the reply; with no board, MISO floats high and EVERY byte reads
 * 0xFF, so the reply's check_sum is 0xFFFF. We therefore require both:
 *   1. the reply is not entirely 0xFF (an absent board reads all-ones), and
 *   2. the reply's check_sum is not 0xFFFF.
 * This keeps the robot firmware's behaviour for a populated robot (where the frame
 * is never all-ones) while still reporting a missing board. */
static bool board_resend(uint8_t board)
{
    int b = board * 3;
    host_SMS_t frame;
    SMS_host_t rx;
    frame.start = START_FIELD;
    frame.mode  = MODE_FIELD;

    servo_cmd_sub_t *sub[3] = { &frame.s1, &frame.s2, &frame.s3 };
    for (int j = 0; j < 3; j++) {
        sub[j]->mode     = sh_mode[b + j];
        sub[j]->position = sh_posdd[b + j];
        sub[j]->torque   = sh_cur[b + j];
        sub[j]->kp = 0;      /* loop gains come from sms_config, not the frame */
        sub[j]->kd = 0;
    }
    frame.check_sum = 0;

    memset(&rx, 0, sizeof rx);
    if (spi_xfer(board, (uint8_t *)&frame, (uint8_t *)&rx) != ESP_OK)
        return false;

    /* An absent board floats MISO high -> the whole reply reads 0xFFFF.
     * Test the raw bytes rather than struct fields so padding/alignment
     * cannot hide an all-ones frame. */
    const uint8_t *raw = (const uint8_t *)&rx;
    bool all_ones = true;
    for (size_t i = 0; i < sizeof rx; i++) {
        if (raw[i] != 0xFF) { all_ones = false; break; }
    }
    if (all_ones) return false;
    if (rx.check_sum == 0xFFFF) return false;

    servo_fb_sub_t *fb[3] = { &rx.s1, &rx.s2, &rx.s3 };
    for (int j = 0; j < 3; j++) {
        fb_posdd[b + j] = fb[j]->position;   /* RAW AT32 deci-degrees */
        fb_cur[b + j]   = fb[j]->torque;     /* present motor current, mA */
        fb_res[b + j]   = fb[j]->res;        /* NTC temperature, 0.1 degC, in the low 16 bits */
        fb_status[b + j] = rx.status;
        fb_valid[b + j] = true;
    }
    return true;
}

bool driver_board_direct(uint16_t servo, uint16_t mode, uint16_t pos_dd, int16_t cur_mA)
{
    if (servo < 1 || servo > 12) return false;
    if (pos_dd > (uint16_t)db_scale_dd) pos_dd = (uint16_t)db_scale_dd;

    int idx = (int)servo - 1;
    sh_mode[idx]  = mode;
    sh_posdd[idx] = pos_dd;
    sh_cur[idx]   = cur_mA;
    return board_resend((uint8_t)(idx / 3));
}

bool driver_board_poll(uint16_t servo)
{
    if (servo < 1 || servo > 12) return false;
    return board_resend((uint8_t)((servo - 1) / 3));
}

/* Diagnostic: send the current shadow frame and hand back the raw reply. */
bool driver_board_raw_reply(uint16_t servo, uint8_t out[36])
{
    if (servo < 1 || servo > 12 || !out) return false;
    int idx = (int)servo - 1;
    int b = (idx / 3) * 3;

    host_SMS_t frame;
    frame.start = START_FIELD;
    frame.mode  = MODE_FIELD;
    servo_cmd_sub_t *sub[3] = { &frame.s1, &frame.s2, &frame.s3 };
    for (int j = 0; j < 3; j++) {
        sub[j]->mode     = sh_mode[b + j];
        sub[j]->position = sh_posdd[b + j];
        sub[j]->torque   = sh_cur[b + j];
        sub[j]->kp = 0;
        sub[j]->kd = 0;
    }
    frame.check_sum = 0;

    memset(out, 0, 36);
    return spi_xfer((uint8_t)(idx / 3), (uint8_t *)&frame, out) == ESP_OK;
}

uint16_t driver_board_present_pos_dd(uint16_t servo)
{
    if (servo < 1 || servo > 12) return 0;
    return fb_posdd[servo - 1];
}

uint32_t driver_board_present_res(uint16_t servo)
{
    if (servo < 1 || servo > 12) return 0;
    return fb_res[servo - 1];
}

uint16_t driver_board_present_status(uint16_t servo)
{
    if (servo < 1 || servo > 12) return 0;
    return fb_status[servo - 1];
}

int16_t driver_board_present_cur_mA(uint16_t servo)
{
    if (servo < 1 || servo > 12) return 0;
    return fb_cur[servo - 1];
}

/* ---- AT32 sms_config parameter access over SPI -------------------------- */

static bool cfg_xfer(uint8_t board, uint16_t op, uint16_t servo_index,
                     uint16_t param_id, float value, SMS_host_t *rx_out)
{
    cfg_frame_t f;
    SMS_host_t rx;
    memset(&f, 0, sizeof f);
    f.start = START_CONFIG;
    f.op = op; f.servo_index = servo_index;
    f.param_id = param_id; f.value = value;
    if (spi_xfer(board, (uint8_t *)&f, (uint8_t *)&rx) != ESP_OK)
        return false;
    if (rx_out) *rx_out = rx;
    return true;
}

/* The AT32 loads its reply into the frame clocked out on the NEXT
 * transaction, so every request is a request/NOP transaction pair. */
static bool cfg_request(uint8_t board, uint16_t op, uint16_t servo_index,
                        uint16_t param_id, float value, float *out)
{
    SMS_host_t rx;
    if (!cfg_xfer(board, op, servo_index, param_id, value, NULL)) return false;
    esp_rom_delay_us(500);                       /* let the AT32 IRQ run   */
    if (!cfg_xfer(board, CFG_OP_NOP, 0, 0, 0, &rx)) return false;
    if (rx.status != START_CONFIG) return false; /* not a config response  */
    if (rx.s1.position != param_id) return false;/* echo mismatch          */
    if (out) memcpy(out, &rx.s1.res, sizeof(float));
    return true;
}

static bool valid_servo(uint16_t servo, uint16_t id, uint16_t id_count)
{
    if (servo < 1 || servo > 12) return false;
    return id < id_count;
}

bool driver_board_set_param(uint16_t servo, uint16_t param_id, float value)
{
    if (!valid_servo(servo, param_id, DB_PARAM_COUNT)) return false;
    int p = (int)servo - 1;
    return cfg_request((uint8_t)(p / 3), CFG_OP_SET,
                       (uint16_t)(p % 3), param_id, value, NULL);
}

bool driver_board_get_param(uint16_t servo, uint16_t param_id, float *out)
{
    if (!valid_servo(servo, param_id, DB_PARAM_COUNT)) return false;
    int p = (int)servo - 1;
    return cfg_request((uint8_t)(p / 3), CFG_OP_GET,
                       (uint16_t)(p % 3), param_id, 0, out);
}

bool driver_board_get_live(uint16_t servo, uint16_t live_id, float *out)
{
    if (!valid_servo(servo, live_id, DB_LIVE_COUNT)) return false;
    int p = (int)servo - 1;
    return cfg_request((uint8_t)(p / 3), CFG_OP_GET_LIVE,
                       (uint16_t)(p % 3), live_id, 0, out);
}
