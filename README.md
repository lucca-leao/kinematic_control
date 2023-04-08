# kinematic_control
Implementation of joint space (moveJ) and cartesian (moveL) space controllers of a 6DoF robotic manipulator. moveJ uses a quintic polynomial to generate trajectories for each individual joint. moveL uses the inverse Jacobian matrix that correlates Cartesian space velocities to Joint space velocities in a feedback loop.

Generally speaking, moveJ results in smoother trajectories for joint positions and unpredictable trajectories in task space, whereas moveL allows for complete control over the task space trajectory, but is prone to singularity errors. The following image exemplifies the difference between the two:

![traj](https://user-images.githubusercontent.com/32180331/230729768-e36252c7-6b98-4957-a60f-814da3e57a29.png)

## Trajectory definition
The moveL control loop allows the definition of any end-effector Cartesian trajectory, as long as it's defined by a continuous and differentiable function.

### Circle
![image](https://user-images.githubusercontent.com/32180331/230730420-7ce1e60c-716a-4954-a804-d2ade0e481eb.png)
![image](https://user-images.githubusercontent.com/32180331/230730437-40a2b516-c744-45be-a644-ef2c7f7f7a4a.png)

### Square
![image](https://user-images.githubusercontent.com/32180331/230730466-22584997-2b75-4b6b-aa12-82e3dfe7c1b4.png)
![image](https://user-images.githubusercontent.com/32180331/230730472-ec7ba1bf-b444-4c01-aecd-ea10e7b64d76.png)

### Polylines
![image](https://user-images.githubusercontent.com/32180331/230730499-6e195b82-7d98-4ac5-9c87-cc3fe3638b38.png)
![image](https://user-images.githubusercontent.com/32180331/230730508-481be222-428a-4039-bcbd-c382c2c1a0c5.png)
