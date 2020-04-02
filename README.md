# A-star Path Planing and Navigation

# Demo:
![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/astar/robot_navigation.png?raw=true)*<center>Figure 1: A_star search path planing and robot navigation </center>*

## Overall:
This project develop an online and offline implementation of A* search algorithm to generate a 2-D path which navigates while avoiding obstacles, and design a inverse kinematics controller to drive a differential wheeled robot to drive along the path.

## Files:
- A_star_offline.py:
  - implement an offline A* search (assume A* have knowledge of the all obstable positions);

- A_star_online.py:
  - implement an online A* search (robot don't have knowledge of the obstacles until robot's position is next to the obstacles position)

- A_star_online_small_grid.py:
  - each node size change from 1m x 1m to 0.1m x 0.1m to make task for realistic. This is also an online A* algorithm.

- Robot_control.py:
  - Controlled robot navigation base on the path get from A*path planing.

## Detals works:

You can find more detals of the owrk from my [github](https://luxi-huang.github.io/portfolio/A_star/) 
