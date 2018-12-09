# PID Controller project

## Overall structure

Steering angle is controlled by a PID controller, pid_steer, whose parameters are determined with twiddle technique.

Throttle is controlled by another PID controller, pid_throttle, whose parameters are manually tuned.

## How each component of PID effects

Each component of Steering PID controller effects as below.

*P component is proportional to CTE and effects as a centerizer.
The bigger this value is, the quicker the vehicle go back to center when it deviates.
However, if this value is too big, the vehicle will diverge and go out of the way.

*D component is proportinal to dCTE/dt.
This acts as a damper to subdue divergence of the vehicle especially the road is straight.
Therefore, as Kp component gets bigger, Kd also needs to increase so as to effectively work as a damper.
Moreover, this takes effect when the vehicle go into a small curve.
In a small curves, the steering rate controlled only by P is not enough to keep the vehicle in the center of the road. Then dCTE/dt increases and this D component tries to cancel it.

*I component is proportinal to integral of CTE.
This is not so effective as the others, relatively, however, as the vehicle's response is always delayed to the change of road shape, the vehicle always runs the outer side of the road.
Therefore, integral of CTE gets increases/decreases in curves and this component also tries to cancel so as to centerize the vehicle.

As a result, all the PID components acts not only centerize the vehicle in straight lines but also turn the vehicle in curves.
However, appropriate PID coefficients depends on the speed and curvature and there is no value covers all the cases.

## How to choose the coefficients of PID controller

I implemented twiddle technique to choose the hyperparameters.

1. Set the throttle to 0.45 and manually choose the hyperparameters.

2. Define the error function as the average of CTE squared and apply twiddle technique.

The problem is that we cannot evaluate the hyperparameters at once and it takes time to calcurate the averaged CTE squared. I set the timespan to 20000 timesteps, in which the vehicle will go around the course several times.

3. Run the twiddle several times and determined the hyperparameters.

4. Change the throttle from constant to PID controller so that the vehicle accelerates in straight lines and slows down in curves.

The final value chosen by twiddle got bigger than the one I manually choose. This is probably because bigger value tends to steer too much and that results in slowdown of the vehicle and smaller error function.