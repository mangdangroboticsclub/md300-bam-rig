# Bridge firmware

ESP-IDF firmware that turns MangDang's MP4 ESP32 CORE board (ESP32-S3) into a
USB-serial to SPI bridge for the MD-300-150's AT32 driver board. BAM's
MD-300-150 driver (`bam/mangdang/spi.py`) talks to it.

```
host --USB serial (CH340K, UART0 GPIO 43/44, 921600 baud)--> ESP32-S3 --SPI2, 12 MHz--> AT32 (one per module)
```

## Build and flash

ESP-IDF v6.0, here through the `espressif/idf:v6.0` Docker image (run from this
directory; `sdkconfig.defaults` selects the ESP32-S3 and 16 MB flash):

```bash
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v "$PWD":/project -w /project \
    espressif/idf:v6.0 idf.py build
docker run --rm -u $(id -u):$(id -g) --group-add dialout -e HOME=/tmp \
    --device /dev/ttyUSB0 -v "$PWD":/project -w /project \
    espressif/idf:v6.0 idf.py -p /dev/ttyUSB0 flash
```

With a local ESP-IDF install, `idf.py build` and `idf.py -p /dev/ttyUSB0 flash`.
Nothing else may hold the serial port while flashing.

## Wiring

| Signal | GPIO |
|---|---|
| SPI MOSI | 6 |
| SPI CLK | 16 |
| SPI MISO | 17 (internal pull-up) |
| CS, connector CN4 | 7 |
| CS, connector CN5 | 4 |
| CS, connector CN3 | 15 |
| CS, connector CN6 | 5 |
| UART TX / RX | 43 / 44 (on-board CH340K) |

Bus ids follow the connector: CN4 answers as ids 1 to 3, CN5 as 4 to 6, CN3 as
7 to 9 and CN6 as 10 to 12 (module slot `k` = 0 to 3 gives `3k + 1` for id_1,
`3k + 2` for id_2, `3k + 3` for id_3).

The servo connectors are powered straight from the board's 12 V input: the
outputs are live whenever the supply is connected. The board has no supply
voltage sense.

## Serial protocol

Little-endian frames, CRC16-CCITT-FALSE (init `0xFFFF`, poly `0x1021`) over all
bytes except the trailing `crc`:

```
request   10 B: 42 cmd servo pad arg1:u16 arg2:u16 crc:u16
response  14 B: 62 cmd servo status pos_dd:u16 cur_mA:i16 val:f32 crc:u16
```

| cmd | Name | Arguments | `val` |
|---|---|---|---|
| 0 | `PING` | | temperature word |
| 1 | `SET_POS` | arg1 position (deci-degrees), arg2 current cap (mA) | temperature word |
| 2 | `IDLE` | (motor off) | temperature word |
| 3 | `TORQUE` | arg1 signed current (mA) | temperature word |
| 4 | `SET_KP` | arg1, arg2 = float `kp_position` | readback |
| 5 | `SET_KD` | arg1, arg2 = float `kd_position` | readback |
| 6 | `GET_PARAM` | arg1 parameter id | value |
| 7 | `GET_LIVE` | arg1 live id | value |
| 8 | `SET_SCALE` | arg1 scale (deci-degrees) | new scale |
| 9 | `RAW_REPLY` | (diagnostic) | packed MISO statistics |
| 10 | `SET_PARAM` | pad = parameter id, arg1, arg2 = float | readback |
| 11 | `GET_STATUS` | | board status word |

- `status`: 0 OK, 1 bad servo id, 2 bad argument, 3 board or SPI failure,
  5 unknown command. A frame with a bad CRC gets no reply.
- `pos_dd`: raw AT32 deci-degrees, 0 to `scale_dd` (default 3100 =
  `range_position_deg` x 10), centre `scale_dd / 2`. `cur_mA`: signed motor
  current. Every reply carries feedback from the same SPI transaction.
- Temperature word: the raw bits of `val` hold the motor NTC temperature in
  0.1 degC (low 16 bits).
- Parameter ids: 0 `reverse_position_sensor`, 1 `min_position_adc`,
  2 `max_position_adc`, 3 `range_position_deg`, 4 `reverse_motor`,
  5 `kp_position`, 6 `kd_position`, 7 `kp_current`, 8 `kff_current`,
  9 `max_pwm_duty_cycle`, 10 `ki_position`, 11 `control_mode`, 12 `pwm_scale`.
  Writes go to the AT32's RAM only.
- Live ids: 0 `pos_adc`, 1 `cur_adc`, 2 `setpoint_pos_deg`, 3 `present_pos_deg`,
  4 `error_pos_deg`, 5 `max_current_mA`, 6 `setpoint_cur_mA`,
  7 `present_cur_mA`, 8 `error_cur_mA`, 9 `pwm_duty`, 10 `mode`,
  11 `loop_counter`, 12 `temperature_c`.
- Parameter and live reads take a request/NOP pair plus 0.5 ms; a position
  command is one 36-byte SPI frame. The bridge prints `#` text lines at boot
  (first at 115200 baud, then at the bridge baud); hosts skip them.
- The bridge baud is `CONFIG_BAM_UART_BAUD` (menuconfig: BAM Bridge).

## Standalone tester

`tools/bam_bridge.py` (Python 3, `pip install pyserial`) checks the bridge
without BAM:

```bash
python3 tools/bam_bridge.py scan               # which bus ids answer
python3 tools/bam_bridge.py dump --servo 8     # all parameters and live values
python3 tools/bam_bridge.py idle --servo 8     # motor off
```

`--port` may be omitted when only one USB-serial port is present.
