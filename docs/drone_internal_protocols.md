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

Packet latency would be slightly less than 20ms. ~~Less latency is useless anyway due to the PWM frequency the motors are driven at (50 Hz); slightly less may be good though, in order to counter kernel-bound latency.~~
Due to hardware constraints, we might as well go ahead and use a higher bitrate.

## Requirements

- **Little-endian**

## Protocols

Unless specified, everything is **little-endian**.

All packets are structured like so:
- Start byte: 1 byte, constant 0x4F
- Message
- CRC16 of message: 2 bytes
- End byte: 1 byte, constant 0x4F

On the message, bytes 0x4F and 0x7F are escaped by prefixing
a 0x7F byte before them.

CRC16 calculation is done using the following C code, or equivalent:
```c
uint16_t crc16_update(uint16_t crc, uint8_t a)
{
int i;

crc ^= a;
for (i = 0; i < 8; ++i)
{
    if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
    else
        crc = (crc >> 1);
}

return crc;
}
```

### Arduino --> RPi, v0.3

- Counter: 2 bytes (`uint16_t`)
  - Increments on each packet sent; good to measure lost packets and ensure synchronization.
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
- Padding: 27 bytes

= 64 bytes

### RPi --> Arduino, v0.3

- Desired roll/pitch/yaw: 2 bytes (**int16_t**) * 3 axes = 6 bytes
- Desired throttle: 2 bytes (uint16_t)
- Padding: 56 bytes

= 64 bytes
