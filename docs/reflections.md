# The effect each of the P, I, D components. 

## P/Kp

Quantitatively, the effect of Kp was to increase the steering angle in proportion to the CTE.  This was expected.  The qualitative effect was more complex.

Starting with default, untuned parameters, increasing Kp tends to make the car more responsive to errors, but quickly overshoot, oscillate wildly, and crash.  Decreasing made the car less responsive and more likely to simply drive off the track.

Starting with the final, tuned parmaters, increasing Kp tends to make the car more responsive to errors, particularly sharp corners.  With the compensatory effect of Kd and Ki, the oscillation was not as severe, but increasing Kp definitely increased instability.  Decreasing Kp made the car less twitchy and stable, but less reactive, unable to navigate the sharp corners of the track.

Overall, Kp had less of a noticeable effect than I expected, and it was difficult to manually tune this PID controller using most approaches in the literature, as they typically start with Kp, and the car would crash 

## I/Ki

Quantitatively, the effect of Ki was to increase the steering angle in proportion to accumulated CTE.  Increasing Ki corrects for sustained errors more quickly, but increases instability and can induce oscillations. Decreasing Ki increases stability, but requires a sustained error to have any corrective effect.

In the context of the lecture, the suggestion was that Ki was mostly useful for correcting system bias.  I didn't expect the simulator to have bias, so I expected Ki to small.

Qualitatively, increasing Ki helped steer the vehicle more slowly toward the center of the lane, but tended to dampen response time to over-corrections to sharp corners.


## D/Kd

Quantitatively, The effect of Kd was to increase the steering angle in proportion to the change in CTE. Increasing Kd had a corrective effect if the car rapidly veered off-center, and a damping effect as the car corrected, and decreased oscillation.  Decreasing Kd increased instability, and made the car less responsive in the sharp corners.

Overall, Kd had the most noticeable effect on the quality of the controller, which was unexpected.

# Selection of the final hyperparameters 

The final hyperparameters were chosen as the result of a guided search process which utilized manual tuning and the Twiddle algorithm.

I started with a throttle value of 0.05 - 0.55 inversely proportional to the absolute steering value.
I.e. a steering value of 1 or -1, resulted in a throttle of 0.05 -- i.e. the car slows the greater correction is required.
I then followed the manual tuning approach described here (https://robotics.stackexchange.com/a/340)
1. Set all gains to 0.
2. Increase Kd until the system oscillates.
3. Reduce Kd by a factor of 2-4.
4. Set Kp to about 1% of Kd.
5. Increase Kp until oscillations start.
6. Decrease Kp by a factor of 2-4.
7. Set Ki to about 1% of Kp.
8. Increase Ki until oscillations start.
9. Decrease Ki by a factor of 2-4.

I found this to be more intuitive than the manual tuning process documented on Wikipedia [https://en.wikipedia.org/wiki/PID_controller], 
because tuning Kd had a greater impact on the initial quality of the controller output in terms of being able to negotiate the sharper corners.

At this speed, it was straightforward to find a set of values that would drive around the whole track.

From there, I performed the following optimization steps:
1. Use Twiddle optimization to reduce the average CTE.  I used an initial step size of 10% of the parameter value.
2. Increase the maximum throttle by 0.5
3. Repeat

Using this approach I was able to increase the maximum throttle up to .85.  The resulting control is rather jerky, but I was not able to effectively smooth out the oscillations without the vehicle crashing.
