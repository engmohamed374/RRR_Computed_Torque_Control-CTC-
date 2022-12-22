# RRR_Computed_Torque_Control(CTC)
Control of a 3DOF RRR serial manipulator.
## Computed Tourque Control 
A well-known robotic controller based on the nonlinear control technique feedback linearization. Simply it "theoretically" eliminates the system non-linearities represented in the M, C, and G matrices in the robotic equation of motion:

![robotic eqn. of motion](https://github.com/engmohamed374/RRR_Computed_Torque_Control-CTC-/blob/main/eqn.gif "robotic eqn. of motion")

As shown in the figure below it is easy to understand that the control cosists of two loops, the inner loop which is responsiple of eliminating the systemm nonlinearities, and the outer loop which is responsible of the motion control. The inner loop is the application of the feedback linearization technique while the outer loop is just a linear control technique known as PID control.

![CTC](https://github.com/engmohamed374/RRR_Computed_Torque_Control-CTC-/blob/main/CTC.png "CTC")

## How To Use
Just run the file named parameters_Values, It shall simulate the system and plot the results which are the position, velocity, acceleration, and thier errors for each link . It is important to mention that the PID controller used in this simulation is a Simulink block and its parameters had been already tuned. You can also replace it with your own PID, but I chose to use the Simulink block because it can be tuned by just one click.