Relating to the specific ROSbot and gazebo simulations, widening the range and tuning the PID controller revealed a significant limitation: the right sensor only detected deviations after the robot had already started to deviate from its intended path.
This reactive response resulted in less effective edge following.
Hence, a different approach can be used with a different robot to get desired output

PID Controller Tuning
The PID controller was tuned iteratively to achieve stable and accurate edge following.
Initially, the gain values were set to Kp = 0.5, Ki = 0.0, and Kd = 0.0. However, these values resulted in oscillatory behavior.
Incremental adjustments to Kp​ were made, with values of 0.6 and 0.7 improving the response. At Kp = 0.8, the performance was further enhanced, although a steady-state error was observed.
To eliminate the steady-state error, a small integral gain (Ki=0.001) was introduced.
To prevent integral windup, a condition was implemented to add error to the integral term only if the accumulated error remained below 0.2.
Finally, to improve stability and reduce overshooting, the derivative gain (Kd​) was set to 0.04. This adjustment was particularly effective due to the robot’s slow speed, where excessive derivative action was unnecessary.
The final PID gain values were determined as follows:
Proportional gain (Kp​): 0.8
Integral gain (Ki​): 0.001
Derivative gain (Kd​): 0.04

A different tuning approach might be necessary in a different environment. 
