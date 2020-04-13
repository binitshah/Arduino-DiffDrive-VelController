Arduino Differential Drive Velocity Control Library
=================

Full duplex control over serial of a differential drive robot through velocity commands, returning integrated SE2 odometry for motion updates in your localization filter.

Setup
-----

We're using a knock off Arduino Uno for the micrcontroller, a [dual MC33926](https://www.pololu.com/product/1213) from Pololu as the motor controller, and two [25D motors w/encoders](https://www.pololu.com/product/4866) also from Pololu.

| Arduino Uno | Motor Controler | Encoders | Descriptions                                                                                            |
|-------------|-----------------|----------|---------------------------------------------------------------------------------------------------------|
| GND         | GND             |          | Ground.                                                                                                 |
| 5V          | VDD             |          | Logic Power.                                                                                            |
| D4          | M1IN2           |          | Motor 1 Directional pin 1. Used to control spin direction of motor. 3 stage forward, backward, stop     |
| D5          | M1IN1           |          | Motor 1 Directional pin 2. Used to control spin direction of motor. 3 stage forward, backward, stop     |
| D6          | PWM M1D2        |          | Motor 1 pulse width modulation. Used to control speed of motor spin.                                    |
| D7          | M1SF            |          | Motor 1 Status Flag, goes LOW if over-current (short circuit) or over-temperature.                      |
| A0          | M1FB            |          | Motor 1 current sensing. 525 mV per amp flowing to motor.                                               |
| D8          | EN              |          | Enable, could be hard break, both motors.                                                               |
| D9          | SLEW            |          | High speed PWM, SLEW high to get finer control. Low speed PWM, Slew LOW and doesn't give finer control. |
| D10         | INV             |          | Inverts directional pins for both motors.                                                               |
| D12         | M2IN2           |          | Motor 2 Directional pin 1. Used to control spin direction of motor. 3 stage forward, backward, stop     |
| D13         | M2IN1           |          | Motor 2 Directional pin 2. Used to control spin direction of motor. 3 stage forward, backward, stop     |
| D11         | PWM M2D2        |          | Motor 2 pulse width modulation. Used to control speed of motor spin.                                    |
| A2          | M2SF            |          | Motor 2 Status Flag, goes LOW if over-current (short circuit) or over-temperature.                      |
| A1          | M2FB            |          | Motor 2 current sensing. 525 mV per amp flowing to motor.                                               |
|             | GND             | M1 GND   | M1 == LEFT Motor GND                                                                                    |
|             | VDD             | M1 5V    | M1 == LEFT Motor 5V                                                                                     |
| D2          |                 | M1 ENC A | Motor 1's encoder A trigger.                                                                            |
| D3          |                 | M1 ENC B | Motor 1's encoder B trigger.                                                                            |
|             | GND             | M2 GND   | M1 == LEFT Motor GND                                                                                    |
|             | VDD             | M2 5V    | M1 == LEFT Motor 5V                                                                                     |
| A4          |                 | M2 ENC A | Motor 2's encoder A trigger.                                                                            |
| A5          |                 | M2 ENC B | Motor 2's encoder B trigger.                                                                            |

Other Setups
------------

See the other branches for other setups and pin outs.
