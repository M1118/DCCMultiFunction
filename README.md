# DCC Multi Device Function decoder
DCC Multi Device controller is an Arduino DCC function decoder that
can control up to 3 servos, 1 stepper motor, one DC motor using PWM
and 4 lights with various different lighting effects and directional
control. The choice of which of these are controlled is based on
the use of function keys; the DCC throttle controls the speed of
movement of each of these and the direction of movement is controlled
by the normal directional controls.

Configuration variables for each servo allow the two end points of
travel to be set, the speed at which the servo will move between
these two end points, the startup mode of the servo and the function
number associated with each servo.

The stepper motor allows the configuration of the number of steps
per revolution, the gearbox ratio, the speed at full throttle and
the function number associated with the stepper motor. The stepper
motor may be used in a free rotational mode or constrained between
two end points.

The DC motor control allows the setting of the maximum speed and
the function number that activates the motor.

Each of the 4 light outputs supports 8 lighting effects and can be
directionally dependent.

There is an additional “soft start” function that can delay the
application of power to the servos in order to reduce the initial
power requirements of the decoder and remove intial “hunting” of
the servos whilst the processor starts up.

The decoder is based on an Arduino Nano.

