# RD-03D Radar Chart Sketch

This Processing sketch reads serial data and renders radar detections on a polar chart.

## How to use

1. Open Processing.
2. Open the sketch folder `RD03D_RadarChart`.
3. Open `RD03D_RadarChart.pde`.
4. Set `serialPortName` if you do not want to use the first available serial port.
5. Set `baudRate` to the correct RD-03D UART baud.
6. Run the sketch.

## Packet format

The sketch parses a binary RD-03D frame using the provided example capture.

Assumed packet structure (30 bytes):

| Byte | Value | Description |
| --- | --- | --- |
| 0 | `0xAA` | Frame header start |
| 1 | `0xFF` | Frame header next |
| 2 | `0x03` | Packet type low byte |
| 3 | `0x00` | Packet type high byte |
| 4-11 | Target 1 block | X1, Y1, velocity, reserved, resolution |
| 12-19 | Target 2 block | X2, Y2, velocity, reserved, resolution |
| 20-27 | Target 3 block | X3, Y3, velocity, reserved, resolution |
| 28 | `0x55` | Frame tail start |
| 29 | `0xCC` | Frame tail end |

### Target block layout

For each target block:

| Byte offset | Field | Description |
| --- | --- | --- |
| 0-1 | `X` | X coordinate, special signed magnitude encoding |
| 2-3 | `Y` | Y coordinate, special signed magnitude encoding |
| 4-5 | `Velocity` | Velocity in cm/s, special signed magnitude encoding |
| 6-7 | `Resolution` | Resolution in mm, little-endian |

### Field interpretation

- `X`, `Y`: 16-bit signed magnitude values in millimeters.
  - The MSB of the high byte is the sign bit.
  - The remaining 15 bits are the absolute magnitude.
  - This is not two's complement.
- `Velocity`: 16-bit signed magnitude value in cm/s.
- `Resolution`: 16-bit little-endian value in millimeters.
- `VALID`: 1 if X or Y is nonzero, else 0.

## Customizing the parser

If your RD-03D packet structure differs:

1. Open `RD03D_RadarChart.pde`.
2. Update `parseBinaryPacket(byte[] packet)`.
3. Adjust `rangeUnitScale` and `velocityUnitScale` to match your sensor units.

## Notes

- The display renders a polar radar grid with configurable FOV and maximum range.
- Detections are drawn as points with a velocity vector overlay.
- The HUD shows packet counts and parse errors for debugging.
