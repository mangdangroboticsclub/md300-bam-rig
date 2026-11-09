/* bam_bridge.c  -  minimal BAM identification firmware for the AT32F413
 * servo driver boards (SCS0009-class servos).
 *
 * The ESP32 acts as a serial<->SPI bridge with a small binary protocol that
 * maps 1:1 onto the operations the BAM actuator pipeline needs:
 *
 *   set_goal_position(rad) -> CMD_SET_POS  (position mode + current cap)
 *   set_torque_enable(on)   -> CMD_SET_POS / CMD_IDLE
 *                              (IDLE = motor off = the torque-free drop that
 *                               lift_and_drop relies on)
 *   set_torque(mA)          -> CMD_TORQUE   (raw current; spin tests)
 *   set_p_gain(kp)          -> CMD_SET_KP   (writes sms_config kp_position)
 *   set_d_gain(kd)          -> CMD_SET_KD   (writes sms_config kd_position)
 *   read_present_position   -> pos_dd field of EVERY response (0.1 deg)
 *   read_present_current    -> cur_mA field of EVERY response (signed mA)
 *   read_pwm_duty           -> CMD_GET_LIVE DB_LIVE_PWM_DUTY
 *   set/get any parameter   -> CMD_SET_PARAM (pad byte = id) / CMD_GET_PARAM
 *                              (ki_position=10, control_mode=11 need AT32 fw >= 0.0.8)
 *   read temperature        -> val of PING/SET_POS/IDLE/TORQUE replies carries the
 *                              feedback "res" word (u32 bits): NTC temp in 0.1 degC
 *   read board status word  -> CMD_GET_STATUS
 *   diagnostics             -> CMD_RAW_REPLY (raw MISO bytes)
 *
 * Frames are little-endian with CRC16-CCITT-FALSE over all bytes except the
 * trailing crc field:
 *
 *   request   10 bytes:  42 cmd servo pad arg1:u16 arg2:u16 crc:u16
 *             (pad = param id for CMD_SET_PARAM, 0 otherwise)
 *   response  14 bytes:  62 cmd servo status pos_dd:u16 cur_mA:i16 val:f32 crc:u16
 *
 *   status: 0 OK, 1 bad servo, 3 board/SPI failure, 5 unknown command
 *   pos_dd: RAW AT32 deci-degrees 0..scale_dd (scale_dd = range_position_deg*10,
 *           default 3100 = 310.0 deg). The host converts to rad.
 *   val:    live value (GET_LIVE), param readback (GET_PARAM/KP/KD), or the
 *           resulting host scale (SET_SCALE).
 *
 * Every servo command refreshes the feedback for the servo's whole board in
 * the same SPI transaction, so the response always carries fresh position +
 * current. Timestamps are taken on the HOST clock (BAM record.py convention);
 * see tools/bam_bridge.py for the reference host driver.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver_board.h"

#define TAG "BAMBRIDGE"

/* ---- serial link (CH340K USB-UART on U0TXD/U0RXD) ---------------------- */
#define BAM_UART_PORT    UART_NUM_0
#define BAM_UART_TX      43
#define BAM_UART_RX      44
/* Bridge baud. Configurable via menuconfig (BAM Bridge > BAM_UART_BAUD).
 * The boot banner is ALWAYS printed at 115200 first, so a default
 * `idf.py monitor` shows a readable message before the baud switch. */
#ifdef CONFIG_BAM_UART_BAUD
#define BAM_UART_BAUD    CONFIG_BAM_UART_BAUD
#else
#define BAM_UART_BAUD    921600
#endif
#define BAM_UART_RX_BUF  256
#define BAM_UART_TX_BUF  256

/* ---- frame layout ------------------------------------------------------- */
#define BAM_MAGIC_REQ   0x42   /* 'B' */
#define BAM_MAGIC_RSP   0x62   /* 'b' */

enum {
    CMD_PING      = 0,   /* refresh feedback, no setpoint change      */
    CMD_SET_POS   = 1,   /* position mode: arg1=pos_dd, arg2=cur cap mA */
    CMD_IDLE      = 2,   /* motor off (torque disable)                 */
    CMD_TORQUE    = 3,   /* torque mode: arg1=signed cur mA             */
    CMD_SET_KP    = 4,   /* arg1|arg2<<16 = float kp_position          */
    CMD_SET_KD    = 5,   /* arg1|arg2<<16 = float kd_position          */
    CMD_GET_PARAM = 6,   /* arg1 = param id (0..DB_PARAM_COUNT-1), val = readback */
    CMD_GET_LIVE  = 7,   /* arg1 = live id (0..DB_LIVE_COUNT-1), val = value     */
    CMD_SET_SCALE = 8,   /* arg1 = host scale deci-deg, val = new scale */
    CMD_RAW_REPLY = 9,   /* diagnostic: report the last 36 raw MISO bytes */
    CMD_SET_PARAM = 10,  /* pad = param id, arg1|arg2<<16 = float, val = readback */
    CMD_GET_STATUS = 11, /* val = raw 16-bit status word of the board's feedback frame */
    CMD_COUNT
};

/* status codes */
#define BAM_OK         0
#define BAM_ERR_SERVO  1
#define BAM_ERR_ARG    2
#define BAM_ERR_BOARD  3
#define BAM_ERR_CRC    4
#define BAM_ERR_CMD    5

#pragma pack(push,1)
typedef struct { uint8_t  magic, cmd, servo, pad; uint16_t arg1, arg2; uint16_t crc; } bam_req_t;  /* 10 B */
typedef struct { uint8_t  magic, cmd, servo, status; uint16_t pos_dd; int16_t cur_mA; float val; uint16_t crc; } bam_rsp_t; /* 14 B */
#pragma pack(pop)
_Static_assert(sizeof(bam_req_t) == 10, "bam_req_t size");
_Static_assert(sizeof(bam_rsp_t) == 14, "bam_rsp_t size");

static QueueHandle_t uart_evq;
static uint8_t  rx_buf[10];
static uint8_t  rx_len = 0;

/* ---- CRC16-CCITT-FALSE (init 0xFFFF, poly 0x1021, MSB-first) ----------- */
static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ---- command execution -------------------------------------------------- */
static uint32_t u16_to_f32_bits(uint16_t lo, uint16_t hi)
{
    return (uint32_t)lo | ((uint32_t)hi << 16);
}

static bam_rsp_t exec_req(const bam_req_t *req)
{
    bam_rsp_t rsp;
    memset(&rsp, 0, sizeof rsp);
    rsp.magic  = BAM_MAGIC_RSP;
    rsp.cmd    = req->cmd;
    rsp.servo  = req->servo;
    rsp.status = BAM_OK;

    bool ok_servo = (req->servo >= 1 && req->servo <= 12);

    switch (req->cmd) {
    case CMD_PING:
        /* resend the board frame so the cached feedback is fresh */
        if (ok_servo && !driver_board_poll(req->servo))
            rsp.status = BAM_ERR_BOARD;
        break;

    case CMD_SET_POS:
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (!driver_board_direct(req->servo, DB_MODE_POSITION,
                                 req->arg1, (int16_t)req->arg2))
            rsp.status = BAM_ERR_BOARD;
        break;

    case CMD_IDLE:
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (!driver_board_direct(req->servo, DB_MODE_IDLE,
                                 driver_board_present_pos_dd(req->servo), 0))
            rsp.status = BAM_ERR_BOARD;
        break;

    case CMD_TORQUE:
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (!driver_board_direct(req->servo, DB_MODE_TORQUE,
                                 (uint16_t)(driver_board_get_scale_dd() / 2.0f),
                                 (int16_t)req->arg1))
            rsp.status = BAM_ERR_BOARD;
        break;

    case CMD_SET_KP:
    case CMD_SET_KD: {
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        uint32_t bits = u16_to_f32_bits(req->arg1, req->arg2);
        float v; memcpy(&v, &bits, sizeof v);
        uint16_t pid = (req->cmd == CMD_SET_KP) ? DB_PARAM_KP_POSITION
                                                : DB_PARAM_KD_POSITION;
        if (!driver_board_set_param(req->servo, pid, v))
            rsp.status = BAM_ERR_BOARD;
        else
            rsp.val = v;
        break;
    }

    case CMD_GET_PARAM: {
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (req->arg1 >= DB_PARAM_COUNT) { rsp.status = BAM_ERR_ARG; break; }
        float v = 0.0f;
        if (!driver_board_get_param(req->servo, req->arg1, &v))
            rsp.status = BAM_ERR_BOARD;
        else
            rsp.val = v;
        break;
    }

    case CMD_SET_PARAM: {
        /* any sms_config parameter, RAM only (nothing is saved to the AT32
         * flash). The id travels in the request's pad byte so both float
         * halves fit in arg1/arg2. */
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (req->pad >= DB_PARAM_COUNT) { rsp.status = BAM_ERR_ARG; break; }
        uint32_t bits = u16_to_f32_bits(req->arg1, req->arg2);
        float v; memcpy(&v, &bits, sizeof v);
        float rb = 0.0f;
        if (!driver_board_set_param(req->servo, req->pad, v) ||
            !driver_board_get_param(req->servo, req->pad, &rb))
            rsp.status = BAM_ERR_BOARD;
        else
            rsp.val = rb;
        break;
    }

    case CMD_GET_STATUS:
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (!driver_board_poll(req->servo)) { rsp.status = BAM_ERR_BOARD; break; }
        rsp.val = (float)driver_board_present_status(req->servo);
        break;

    case CMD_GET_LIVE: {
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        if (req->arg1 >= DB_LIVE_COUNT) { rsp.status = BAM_ERR_ARG; break; }
        float v = 0.0f;
        if (!driver_board_get_live(req->servo, req->arg1, &v))
            rsp.status = BAM_ERR_BOARD;
        else
            rsp.val = v;
        break;
    }

    case CMD_SET_SCALE:
        driver_board_set_scale_dd((float)req->arg1);
        rsp.val = driver_board_get_scale_dd();
        break;

    case CMD_RAW_REPLY: {
        /* Diagnostic: how many of the 36 MISO bytes are 0xFF, and a few
         * samples. 36/36 means no board (line pulled up); a wildly varying
         * count means MISO is floating. Either way the host can tell WHY a
         * board is not answering. */
        if (!ok_servo) { rsp.status = BAM_ERR_SERVO; break; }
        uint8_t raw[36];
        if (!driver_board_raw_reply(req->servo, raw)) {
            rsp.status = BAM_ERR_BOARD;
            break;
        }
        int ones = 0;
        for (int i = 0; i < 36; i++) if (raw[i] == 0xFF) ones++;
        /* pack into val: count of 0xFF bytes, plus the first two words */
        uint32_t packed = (uint32_t)ones
                        | ((uint32_t)raw[0] << 8)
                        | ((uint32_t)raw[1] << 16)
                        | ((uint32_t)raw[2] << 24);
        memcpy(&rsp.val, &packed, sizeof rsp.val);
        break;
    }

    default:
        rsp.status = BAM_ERR_CMD;
        break;
    }

    /* attach fresh feedback (direct writes already refreshed the cache).
     * A board that did not answer leaves the cache invalid - report that
     * instead of shipping stale/0xFFFF values as if they were real. */
    if (ok_servo) {
        if (driver_board_feedback_valid(req->servo)) {
            rsp.pos_dd = driver_board_present_pos_dd(req->servo);
            rsp.cur_mA = driver_board_present_cur_mA(req->servo);
            /* commands that carry no other value ship the feedback "res"
             * word (NTC temperature, 0.1 degC) as the raw bits of val, so
             * the host gets the temperature for free with every sample. */
            switch (req->cmd) {
            case CMD_PING: case CMD_SET_POS: case CMD_IDLE: case CMD_TORQUE: {
                uint32_t res = driver_board_present_res(req->servo);
                memcpy(&rsp.val, &res, sizeof rsp.val);
                break;
            }
            default: break;
            }
        } else if (rsp.status == BAM_OK) {
            rsp.status = BAM_ERR_BOARD;
            rsp.pos_dd = 0;
            rsp.cur_mA = 0;
        }
    }
    return rsp;
}

/* ---- byte-by-byte frame assembly (runs in the bridge task only) --------- */
static void on_byte(uint8_t b)
{
    if (rx_len == 0) {
        if (b != BAM_MAGIC_REQ) return;   /* skip junk until a magic byte */
    }
    rx_buf[rx_len++] = b;
    if (rx_len == sizeof(bam_req_t)) {
        rx_len = 0;
        bam_req_t req;
        memcpy(&req, rx_buf, sizeof req);
        if (crc16(rx_buf, 8) != req.crc) return;   /* resync on next byte */
        bam_rsp_t rsp = exec_req(&req);
        rsp.crc = crc16((uint8_t *)&rsp, 12);
        uart_write_bytes(BAM_UART_PORT, &rsp, sizeof rsp);
    }
}

static void bridge_task(void *arg)
{
    (void)arg;
    uart_event_t ev;
    uint8_t buf[64];
    for (;;) {
        if (xQueueReceive(uart_evq, &ev, portMAX_DELAY) != pdTRUE) continue;
        switch (ev.type) {
        case UART_DATA: {
            int n = uart_read_bytes(BAM_UART_PORT, buf, sizeof buf, 0);
            for (int i = 0; i < n; i++) on_byte(buf[i]);
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            uart_flush_input(BAM_UART_PORT);
            xQueueReset(uart_evq);
            rx_len = 0;
            break;
        default:
            break;
        }
    }
}

static void uart_init(void)
{
    uart_config_t c = {
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BAM_UART_PORT, BAM_UART_RX_BUF,
                                        BAM_UART_TX_BUF, 16, &uart_evq, 0));
    ESP_ERROR_CHECK(uart_set_pin(BAM_UART_PORT, BAM_UART_TX, BAM_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Announce at the bootloader's 115200 baud first, so a default
     * `idf.py monitor` shows a readable message instead of garbage, then
     * switch to the bridge baud. */
    c.baud_rate = 115200;
    ESP_ERROR_CHECK(uart_param_config(BAM_UART_PORT, &c));
    char pre[160];
    snprintf(pre, sizeof pre,
             "# BAM bridge booted. Bridge baud = %d. To see the rest:\r\n"
             "#   idf.py monitor --monitor-baud %d\r\n"
             "#   (or use tools/bam_bridge.py at the same baud)\r\n",
             (int)BAM_UART_BAUD, (int)BAM_UART_BAUD);
    uart_write_bytes(BAM_UART_PORT, pre, strlen(pre));
    ESP_ERROR_CHECK(uart_wait_tx_done(BAM_UART_PORT, pdMS_TO_TICKS(200)));

    c.baud_rate = BAM_UART_BAUD;
    ESP_ERROR_CHECK(uart_param_config(BAM_UART_PORT, &c));
    char boot[96];
    snprintf(boot, sizeof boot,
             "# BAM bridge ready: scale=%.1f deg, baud=%d 8N1\r\n",
             driver_board_get_scale_dd() / 10.0f, (int)BAM_UART_BAUD);
    uart_write_bytes(BAM_UART_PORT, boot, strlen(boot));
    ESP_ERROR_CHECK(uart_wait_tx_done(BAM_UART_PORT, pdMS_TO_TICKS(200)));
}

void app_main(void)
{
    driver_board_init();
    uart_init();   /* prints 115200 banner, then the bridge-baud ready line */

    xTaskCreate(bridge_task, "bam_bridge", 4096, NULL, 10, NULL);
}
