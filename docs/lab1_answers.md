
What are problem(s) that could occur, when...

1. The lookahead distance is too large?
If the lookahead distance is too large, there is significantly reduced responsiveness to sharp curves, resulting in overshooting and hence higher tracking errors along the planned path. A large lookahead distance also results in larger turn radius, resulting in the robot taking a path that is smooth but smaller in curvature. 

2. The lookahead distance is too small?
A small lookahead distance would result in excessively aggressive corrections to minor deviations from the planned path. This would result in oscillatory motion due to its high sensitivity towards errors, making it difficult to maintain a smooth and stable path. 

3. The linear velocity is too large?
When the linear velocity is too large, there is lesser time for the robot to react and hence causing a delay in steering around curves. This leads to tracking errors and overshoot, as it fails to follow tight curves due to its increased inertia.


4. The lookahead point is to the left or right of the robot, such that $y' \approx 0$?
When y' is approximately 0, the curvature of the robot would theoretically be 0, indicating a straight line motion. Hence the robot might not turn towards the direction of its lookahead point, resulting in tracking errors and potential overshooting when it attempts to correct its path to the desired path later. 


5. The lookahead point is behind the robot, such that $x' < 0$?
When the lookahead point is behind the robot, the movement of the robot becomes temporarily unpredictable. It steers off path momentarily searching for the closest goal pose but eventually rotates back to the desired path. This occurs as the lookahead point is assumed mathematically to be ahead of the robot and should be a positive number. 