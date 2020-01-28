# Drone-ground communications

## Requirements:
- Maximum packet size 32 bytes
- Little-endian!

## Protocols

Unless specified, everything is **little-endian**.

### Ground-to-air

- Header byte: 1 byte
  - Constant value: 0x7F
- Counter: 2 bytes
- Command type + length of command data: 2 byte
    - 0x02: (CNTA) Controller axis state
        - Controller axis state: 1 **SIGNED** byte (`int8_t`) * 4 axis = 4 bytes
    - 0x10: (APPM) Application message
        - Application ID: 1 byte
        - Message length: 1 byte (`uint8_t`)
        - Message: variable (6 - 25 bytes)
    - 0xFF: (HEAR) Heartbeat
        - Timestamp: 4 bytes (`uint32_1`)

### Air-to-ground
- Header byte: 1 byte
  - Constant value: 0x7F
- Counter: 2 bytes
- Report type + length of report: 2 bytes
    - 0x02 (MOTR): Motor speed
        - Motor speed: 2 bytes (`uint16_t`) * 4 motors = 8 bytes
    - 0x03 (ORNT): Orientation
        - Orientation quartenion: 4 bytes (`float`) * 4 axis = 16 bytes
    - 0x04 (ACEL): Acceleration
        - Accelerometer XYZ: 4 bytes (`float`) * 3 axes = 12 bytes
    - 0x10 (APPM): Application message
        - Application ID: 1 byte
        - Message length: 1 byte (`uint8_t`)
        - Message: variable (6 - 25 bytes)
