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

Commands are comprised of space separated keywords in their respective order and are case-insensitive.

| Function | Category  | Target   | Sub-Target | Parameter | Description                                                                     |
| -------- | --------- | -------- | ---------- | --------- | ------------------------------------------------------------------------------- |
| STOP     | -         | -        |            | -         | Perform E-STOP, shutdown all motors immediately then shutdown flight controller |
| SET      | PID       | Y        | P          | float     | Set yaw axis PID proportional gain                                              |
| SET      | PID       | Y        | I          | float     | Set yaw axis PID integral gain                                                  |
| SET      | PID       | Y        | D          | float     | Set yaw axis PID derivative gain                                                |
| SET      | PID       | Y        | CEL        | float     | Set yaw axis PID cumulative error limit                                         |
| SET      | PID       | Y        | IAT        | float     | Set yaw axis PID intgrator active threshold                                     |
| SET      | PID       | PR       | P          | float     | Set pitch/roll axis PID proportional gain                                       |
| SET      | PID       | PR       | I          | float     | Set pitch/roll axis PID integral gain                                           |
| SET      | PID       | PR       | D          | float     | Set pitch/roll axis PID derivative gain                                         |
| SET      | PID       | PR       | CEL        | float     | Set pitch/roll axis PID cumulative error limit                                  |
| SET      | PID       | PR       | IAT        | float     | Set pitch/roll axis PID intgrator active threshold                              |
| SET      | PID       | MLI      |            | int       | Set PID minimum loop interval (microseconds)                                    |
| SET      | TRIM      | H        |            | float     | Set heading trim                                                                |
| SET      | TRIM      | P        |            | float     | Set pitch trim, more positive values will cause drone to nose down              |
| SET      | TRIM      | R        |            | float     | Set roll trim, more positive values will cause drone right wing to come down    |
| SET      | RADIO     | DEADZONE |            | int       | Set radio deadzone                                                              |
| SET      | CONTROL   | YAR      |            | float     | Set control yaw angle rate (degrees / second)                                   |
| SET      | TELEMETRY | INTERVAL |            | INT       | Set telemetry transmission interval (microseconds)                              |
