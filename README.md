## Differential Astar Turtlebot
A* search algorithm to search the tree and to find the optimal path for navigating a  TurtleBot  in a given map environment from a given start point to a given goal point.

## Dependencies For Phase3

The libraries being imported to run the code are: 
1. numpy
2. math
3. pygame
4. sys 
5.python 2.7

## To Run the code for Phase3:
```
python Differential_Astar.py --start 1 3 --goal 9 8 --rpm1 10 --rpm2 20 --theta 0
```
Output Videos are in Phase3/Output_Phase3 folder.

## Dependencies For Phase4

The libraries being imported to run the code are: 
1. numpy
2. math
3. pygame
4. sys 
5. python 2.7
6. Ros Kinetic
7. Gazebo
8. Turtlebot2

## To Run the code for Phase4:
```
export TURTLEBOT_GAZEBO_WORLD_FILE="./map.world"
ROBOT_INITIAL_POSE="-x 4 -y 2 -Y -3.14" roslaunch turtlebot_gazebo turtlebot_world.launch
```

## Simulation 1
```
python Differential_Astar_Turtlebot.py --start 1 3 --goal 9 8 
 ```
## Simulation 2
```
python Differential_Astar_Turtlebot.py --start 1 3 --goal 2 3 
```
## Simultion 3
```
python Differential_Astar_Turtlebot.py --start 1 3 --goal 5 3
```
Output Videos are in Phase4/Output_Phase4 folder.

Because of no controller the turtlebot doesn't follow all the way points exactly.
