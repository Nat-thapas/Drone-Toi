# Drone Toi

A 12-inch STM32 based drone with custom from the ground up flight control.

## Controls

Control is handled by FS-I6X transmitter and FS-IA6B radio receiver. The STM32 flight controller receive data from the radio receiver via the i-BUS protocol (UART 8N1 115200 bps). Each channel is bound to the following:

| Channel | Input                          | Control                          |
| ------- | ------------------------------ | -------------------------------- |
| 1       | Right joystick horizontal axis | Roll                             |
| 2       | Right joystick vertical axis   | Pitch                            |
| 3       | Left joystick vertical axis    | Power                            |
| 4       | Left joystick horizontal axis  | Yaw                              |
| 5       | VRA (left knob)                | -                                |
| 6       | VRB (right knob)               | -                                |
| 7       | SWA (1st switch)               | Disable/enable output            |
| 8       | SWB (2nd switch)               | Enabled/disable flight assists   |
| 9       | SWC (3rd switch)               | Low/med/high control sensitivity |
| 10      | SWD (4th switch)               | Disabled/enabled PID integrator  |

## Telemetry and configuration

The drone have 2 command line interfaces you can use to view the telemetry and configure some configurable values. The interfaces are as follow:

1. Virtual COM port via ST-LINK (USART3 in UART mode)

   To connect to this interface, use a USB cable to connect to ST-LINK using any terminal emulator and set baud rate to 1,500,000 (1.5 Mbps)

2. Wireless serial communication via ESP32 (USART6 in UART mode)

   To connect to this interface, connect to the ESP32 Wi-Fi as configured using `socat` or similar tool in raw mode

After you've connected to one of the interface, the telemetry data will be visible. You can also enter commands to set configuration values as follow:

Commands are comprised of space separated keywords in their respective order and are case-insensitive.

| Function | Category  | Target   | Sub-Target | Parameter | Description                                                                          |
| -------- | --------- | -------- | ---------- | --------- | ------------------------------------------------------------------------------------ |
| STOP     | -         | -        | -          | -         | Perform E-STOP, shutdown all motors immediately then shutdown flight controller      |
| SET      | PID       | Y        | P          | float     | Set yaw axis PID proportional gain                                                   |
| SET      | PID       | Y        | I          | float     | Set yaw axis PID integral gain                                                       |
| SET      | PID       | Y        | D          | float     | Set yaw axis PID derivative gain                                                     |
| SET      | PID       | Y        | CEL        | float     | Set yaw axis PID cumulative error limit                                              |
| SET      | PID       | Y        | IAT        | float     | Set yaw axis PID intgrator active threshold                                          |
| SET      | PID       | PR       | P          | float     | Set pitch/roll axis PID proportional gain                                            |
| SET      | PID       | PR       | I          | float     | Set pitch/roll axis PID integral gain                                                |
| SET      | PID       | PR       | D          | float     | Set pitch/roll axis PID derivative gain                                              |
| SET      | PID       | PR       | CEL        | float     | Set pitch/roll axis PID cumulative error limit                                       |
| SET      | PID       | PR       | IAT        | float     | Set pitch/roll axis PID intgrator active threshold                                   |
| SET      | PID       | MLI      | -          | int       | Set PID minimum loop interval (microseconds)                                         |
| SET      | TRIM      | H        | -          | float     | Set heading trim                                                                     |
| SET      | TRIM      | P        | -          | float     | Set pitch trim, more positive values will cause drone to pitch up (nose up)          |
| SET      | TRIM      | R        | -          | float     | Set roll trim, more positive values will cause drone to roll right (right wing down) |
| SET      | RADIO     | DEADZONE | -          | int       | Set radio deadzone                                                                   |
| SET      | CONTROL   | YAR      | -          | float     | Set control yaw angle rate (degrees / second)                                        |
| SET      | TELEMETRY | INTERVAL | -          | int       | Set telemetry transmission interval (microseconds)                                   |
| SET      | IMU       | LPFA     | YR         | float     | Set IMU yaw rate low-pass filter alpha value                                         |
| SET      | IMU       | LPFA     | PR         | float     | Set IMU pitch rate low-pass filter alpha value                                       |
| SET      | IMU       | LPFA     | RR         | float     | Set IMU roll rate low-pass filter alpha value                                        |

Values set via command are NOT persisted to flash memory and will be lost upon restart.
