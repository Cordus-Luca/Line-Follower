# Line-Follower

Link video: https://www.youtube.com/shorts/HTjjznXwInI (Stiu ca e short, nu am reusit sa il pun ca videoclip normal).

# Used Components:

1. Arduino Uno
2. Zip-ties
3. A LiPo battery
4. Wheels (2)
5. Wires for the line sensor (female - male)
6. QTR-8A reflectance sensor, along with screws
7. Ball caster
8. Extra wires from the kit or lab
9. Chassis
10. Breadboard - medium (400pts)
11. L293D motor driver
12. DC motors (2)

# Logic

The robot uses a sensor to find out its position relative to the line. The sensor returns a value between 0 and 5000 (0 = line is on the right, 5000 = line is on the left) which is mapped in the interval [-2500, 2500].

Proportional-Integral-Derivative (PID) is used to control the speed and direction of the robot.

To control the robot we mainly tuned the kp and kd variables:
  - kp, which controls the current error value, was set to 15
  - kd, which controls how much the rate change of the error affects the output, was set to 30
  
 The robot managed to complete the course in a little over 19 seconds
