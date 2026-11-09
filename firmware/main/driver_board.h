/* driver_board.h
 *
 * Minimal SPI backend for the BAM identification firmware.
 *
 * Ported from the SPI servo driver of MangDang's robot ESP32 firmware: the
 * on-the-wire protocol toward the AT32F413 servo driver boards (one board per
 * leg, 3 servos each) is byte-identical to that driver. Only the operations the BAM bridge needs
 * are kept:
 *   - single-servo direct write (position / torque / idle) with feedback
 *   - sms_config parameter get/set (kp_position, kd_position, ...)
 *   - live control-loop value fetch (pwm duty, loop counter, ...)
 *   - runtime SCS scale
 *
 * IMPORTANT:
 *   In MODE_POSITION the AT32 firmware reads the "torque" field of the command
 *   frame as a MAX CURRENT LIMIT in milliamps (max_current_mA) - a CAP, not a
 *   forced draw. There is no "speed" field.
 *
 * Position units throughout this file: RAW AT32 deci-degrees (0.1 deg per
 * count), 0..db_scale_dd, where db_scale_dd = range_position_deg * 10
 * (default 3100 = 310.0 deg). Centre = db_scale_dd / 2.
 *
 * Servo ID -> board/channel mapping (board = (id-1)/3):
 *   1,2,3    = board 0 (CS CN4)      7,8,9    = board 2 (CS CN3)
 *   4,5,6    = board 1 (CS CN5)      10,11,12 = board 3 (CS CN6)
 */
#ifndef DRIVER_BOARD_H
#define DRIVER_BOARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* servo modes (match the AT32 firmware) */
#define DB_MODE_IDLE     0
#define DB_MODE_POSITION 1
#define DB_MODE_TORQUE   2

/* ---- AT32 sms_config parameter access over SPI --------------------------
 * Parameter ids match the AT32 UART CLI table order. */
enum {
    DB_PARAM_REVERSE_POSITION_SENSOR = 0,
    DB_PARAM_MIN_POSITION_ADC,      /* 1 */
    DB_PARAM_MAX_POSITION_ADC,      /* 2 */
    DB_PARAM_RANGE_POSITION_DEG,    /* 3 */
    DB_PARAM_REVERSE_MOTOR,         /* 4 */
    DB_PARAM_KP_POSITION,           /* 5 */
    DB_PARAM_KD_POSITION,           /* 6 */
    DB_PARAM_KP_CURRENT,            /* 7 */
    DB_PARAM_KFF_CURRENT,           /* 8 */
    DB_PARAM_MAX_PWM_DUTY_CYCLE,    /* 9 */
    DB_PARAM_KI_POSITION,           /* 10  AT32 fw >= 0.0.7: position integral gain (0 = off)   */
    DB_PARAM_CONTROL_MODE,          /* 11  AT32 fw >= 0.0.8: 0 legacy current loop, 1 PID->PWM  */
    DB_PARAM_PWM_SCALE,             /* 12  AT32 fw 0.0.9 */
    DB_PARAM_COUNT
};

/* ---- live control-loop values (same set the AT32 uart_trace prints) ----- */
enum {
    DB_LIVE_POS_ADC = 0,        /* position sensor, raw ADC     */
    DB_LIVE_CUR_ADC,            /* current sense, raw ADC       */
    DB_LIVE_SETPOINT_POS_DEG,   /* commanded position, deg      */
    DB_LIVE_PRESENT_POS_DEG,    /* measured position, deg       */
    DB_LIVE_ERROR_POS_DEG,      /* position error, deg          */
    DB_LIVE_MAX_CURRENT_MA,     /* current cap (position mode)  */
    DB_LIVE_SETPOINT_CUR_MA,    /* current setpoint (torque)    */
    DB_LIVE_PRESENT_CUR_MA,     /* measured motor current, mA   */
    DB_LIVE_ERROR_CUR_MA,       /* current error, mA            */
    DB_LIVE_PWM_DUTY,           /* PWM duty cycle 0..1          */
    DB_LIVE_MODE,               /* 0 idle 1 position 2 torque   */
    DB_LIVE_LOOP_COUNTER,       /* control loop tick counter    */
    DB_LIVE_TEMPERATURE_C,      /* 12 motor NTC temperature, degC (fw >= 0.0.6 NTC build) */
    DB_LIVE_COUNT
};

/* Initialise the SPI bus and the 4 driver-board devices. Call once. */
void driver_board_init(void);

/* Send one servo command frame to the servo's board; the OTHER two servos on
 * that board keep their last commanded values (shadow). pos_dd is the RAW
 * AT32 angle in deci-degrees, 0..db_scale_dd (centre = half). cur_mA is the
 * current cap in position mode or the signed current setpoint in torque mode.
 * Updates the cached feedback for the whole board on success. */
bool driver_board_direct(uint16_t servo /*1..12*/, uint16_t mode,
                         uint16_t pos_dd, int16_t cur_mA);

/* Refresh feedback for this servo's board by resending the last commanded
 * frame (setpoints unchanged). Use driver_board_present_* afterwards. */
bool driver_board_poll(uint16_t servo /*1..12*/);

/* Cached feedback from the last direct write / poll. servo = 1..12. */
uint16_t driver_board_present_pos_dd(uint16_t servo);  /* RAW deci-degrees  */
int16_t  driver_board_present_cur_mA(uint16_t servo);  /* motor current, mA */
uint32_t driver_board_present_res(uint16_t servo);     /* feedback reserved word: low 16 = NTC temp in 0.1 degC */
uint16_t driver_board_present_status(uint16_t servo);  /* feedback status word of the servo's board */

/* True if the servo's board answered on the last transfer. False means the
 * board is absent/unplugged, so the present_* values above are stale. */
bool driver_board_feedback_valid(uint16_t servo);

/* SCS full-scale in AT32 deci-degrees. MUST equal range_position_deg x 10 on
 * the AT32 boards. Default 3100 (310.0 deg); clamped to 100..3600. */
void  driver_board_set_scale_dd(float v);
float driver_board_get_scale_dd(void);

/* servo = 1..12 (global id). Return true on success. */
bool driver_board_set_param(uint16_t servo, uint16_t param_id, float value);
bool driver_board_get_param(uint16_t servo, uint16_t param_id, float *out);
bool driver_board_get_live(uint16_t servo, uint16_t live_id, float *out);

/* Diagnostic: send the current frame to the servo's board and copy the 36
 * raw reply bytes out. Lets the host see exactly what MISO returns (all-ones
 * = no board, noise = floating line). Returns false if the SPI transfer
 * itself failed. */
bool driver_board_raw_reply(uint16_t servo, uint8_t out[36]);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BOARD_H */
