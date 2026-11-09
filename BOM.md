# Bill of materials

## Printed parts

| Part | Count | File |
|---|---|---|
| Side anchor holder (id_1) | 1 | `hardware/side_anchor/side_anchor_holder.STL` |
| Side anchor housing (id_1) | 1 | `hardware/side_anchor/side_anchor_housing.STL` |
| Back portion holder, side anchor (id_1) | 1 | `hardware/side_anchor/V2_back_portion_holder.STL` |
| Geared piece motor holder (id_2, id_3) | 1 | `hardware/coaxial_anchor/geared_piece_motor_holder.STL` |
| Smooth piece motor holder (id_2, id_3) | 1 | `hardware/coaxial_anchor/smooth_piece_motor_holder.STL` |
| Back portion holder, coaxial anchor (id_2, id_3) | 1 | `hardware/coaxial_anchor/V2_back_portion_holder.STL` |
| Motor holder bottom (id_2, id_3) | 1 | `hardware/coaxial_anchor/V2_Motor_Holder_Bottom.STL` |
| Pendulum arm, 10 cm | 1 | `hardware/pendulum/10cm_arm.STL` |
| Pendulum arm, 12.5 cm | 1 | `hardware/pendulum/12_5cm_arm.STL` |
| Pendulum arm, 15 cm | 1 | `hardware/pendulum/15cm_arm.STL` |
| Payload bed (disc, 6 cm diameter, 2 cm high, about 30 g) | 1 | `hardware/pendulum/payload_bed.stl` |

## Fasteners

| Use | Parts |
|---|---|
| id_1 anchor (side) | 4 × M4×50 screws, 4 × M4 nuts, 4 × M3×10 screws, 4 × M3 heat inserts (5.5 × 4 mm) |
| id_2 / id_3 anchor (coaxial) | 4 × M4×50 screws, 4 × M4 nuts, 4 × M3×10 screws, 4 × M3 heat inserts (5.5 × 4 mm) |
| Arm to output, id_1 and id_3 | 5 × M2×4 screws |
| Arm to output, id_2 | 4 × M1.6×6 screws, 4 × M2 nuts as spacers (they keep the arm clear of the housing) |
| Payload bed to arm | 2 × M4×25 screws, 2 × M4 nuts |
| Payload weights | 8 × M8×30 bolts (≈ 15.6 g each), 36 × M8 nuts (≈ 4.7 g each) |

## Other parts

- An MD-300-150 module (AT32 firmware 0.0.9) and an ESP32-S3 board running the
  bridge in `firmware/` (the Mini Pupper v4 board works), with its cable to the
  module and a USB cable to the host.
- A 12 V bench supply and a bench clamp.
- A scale, to weigh the arm and the payload.
