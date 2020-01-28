# Drone RPi-Arduino communications

We want to send 100 serial packets/sec through USB serial port between Raspberry Pi and Arduino (we want to do duplex comms, so 50 in, 50 out, interleaved).

64 bytes per packet for both directions (lower packet sizes make the drivers add artificial delays for some reason).
```
100 packets/s * 64 bytes = 6400 bytes/s
```

At 9600 baud rate, assuming 2 overhead bits (start+stop) and 1 separator bit, useful data speed is:
```
8/11 = X/9600 --> X = (9600*8)/11 ~= 6981 bytes/s
```

Transmission should be doable; used data speed < usable data speed.

Packet latency would be slightly less than 20ms. Less latency is useless anyway due to the PWM frequency the motors are driven at (50 Hz); slightly less may be good though, in order to counter kernel-bound latency.

## Requirements

- Little-endian

## Protocols

### Arduino --> RPi, v0.3

- Header byte: 1 byte
- Counter: 2 bytes (`uint16_t`)
  - Increments on each packet sent; good to measure lost packets and ensure synchronisation.
- Motor speed: 2 bytes (`uint16_t`) * 4 motors = 8 bytes
  - Range taken by the PCA9685 servo controller has 12 bits of resolution.
- Orientation quartenion: ~~8~~ 4 bytes (~~`double`~~ `float` or less) * 4 axis = ~~32~~ 16 bytes (see optimization)
  - Optimization 1 (TO BE CONFIRMED): from observed screenshots from Adafruit, precision is actually
  so **low** that **from the second digit of each decimal** all decimal parts are always one of:
  `0, 125, 250, 375, 500, 625, 750, 875`.
  Not only this fits into a `float` even though the exposed type is a `double`, but we could also encode the
  whole number into a single `uint16_t` where the number occupies the least-significant 9 bits, then the decimal is
  encoded in the next 3 bits (decimal has only 8 = 2^3 possible values! We can encode each of those values in 3 bits).
  - Optimization 2?: If we really need to overdo it, from Optimization 1: the entire number is now encoded in 12 bits,
  and thus we can bitpack the entire quartenion into 12 * 4 = 48 bits
  = 6 bytes!
- Accelerometer XYZ: 4 bytes (`float`) * 3 axes = 12 bytes
- Padding: 25 bytes

= 64 bytes

### RPi --> Arduino, v0.3

- Header byte: 1 byte
  - Constant value: 0x4F
- Desired roll/pitch/yaw: 2 bytes (**int16_t**) * 3 axes = 6 bytes
- Desired throttle: 2 bytes (uint32_t)
- PID constant values: 4 bytes (float) * 3 values * 3 axis = 36 bytes
- Padding: 20 bytes

= 64 bytes
