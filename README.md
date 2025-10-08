# Controls

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

# Commands

| Function | Category  | Target   | Argument | Description                                                                     |
| -------- | --------- | -------- | -------- | ------------------------------------------------------------------------------- |
| STOP     | -         | -        | -        | Perform E-STOP, shutdown all motors immediately then shutdown flight controller |
| SET      | PID       | P        | float    | Set PID proportional gain                                                       |
| SET      | PID       | I        | float    | Set PID integral gain                                                           |
| SET      | PID       | D        | float    | Set PID derivative gain                                                         |
| SET      | PID       | CEL      | float    | Set PID cumulative error limit                                                  |
| SET      | PID       | IAT      | float    | Set PID integrator active threshold                                             |
| SET      | PID       | MLI      | int      | Set PID minimum loop interval (microseconds)                                    |
| SET      | TRIM      | H        | float    | Set heading trim                                                                |
| SET      | TRIM      | P        | float    | Set pitch trim, more positive values will cause drone to nose down              |
| SET      | TRIM      | R        | float    | Set roll trim, more positive values will cause drone right wing to come down    |
| SET      | RADIO     | DEADZONE | int      | Set radio deadzone                                                              |
| SET      | CONTROL   | YSM      | float    | Set control yaw sensitivity multiplier                                          |
| SET      | TELEMETRY | INTERVAL | INT      | Set telemetry transmission interval (microseconds)                              |
