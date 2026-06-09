1. Connect the Balance Sensor This part is the robot's "inner ear." It tells the Pico if it's tipping over.
VCC wire ➞ 3.3V pin on the Pico (This is its power).

GND wire ➞ any GND pin on the Pico (This is its ground).

SCL wire ➞ pin GP1 on the Pico.

SDA wire ➞ pin GP0 on the Pico.

2. Connect the Motor Controller The Pico isn't strong enough to spin the wheels itself. It tells the motor controller (the L298N) what to do, and the controller sends power to the wheels.
Left Motor Control: Connect 3 wires from the Pico's pins GP15, GP13, and GP12 to the L298N's pins labeled ENA, IN1, and IN2.

Right Motor Control: Connect 3 wires from the Pico's pins GP10, GP9, and GP8 to the L298N's pins labeled ENB, IN3, and IN4.

3. Connect the Power and Wheels This is the final step to give your robot life!
Wheels: Plug the two wires from your left motor into the OUT1 and OUT2 screw holes on the L298N. Plug the right motor into OUT3 and OUT4.

Battery:

The red wire (+) from your battery connects to the big power screw hole on the L298N (usually marked +12V or VIN).

The black wire (-) from your battery must connect to TWO places:

The GND screw hole on the L298N.

Any GND pin on your Pico.
