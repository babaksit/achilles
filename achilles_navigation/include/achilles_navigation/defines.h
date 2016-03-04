#ifndef DEFINES_H
#define DEFINES_H

# define M_PI		3.14159265358979323846

//Filter Moving Direction

#define FORWARD_SIDE_LIMIT_I_L 256 //index left
#define FORWARD_SIDE_LIMIT_I_R 427 //index right

#define RIGHT_SIDE_LIMIT_I_L 171 //index left
#define RIGHT_SIDE_LIMIT_I_R 256 //index right

#define LEFT_SIDE_LIMIT_I_L 427 //index left
#define LEFT_SIDE_LIMIT_I_R 512 //index right

#define LASER_DATA_MIN_DIST 20


#define AVG_LASER_DATA_LIMIT 400
#define AVG_LASER_DATA_LIMIT_I_L 171
#define AVG_LASER_DATA_LIMIT_I_R 512

#define MOVING_DIRECTION_LEFT_BOUNDARY 110
#define MOVING_DIRECTION_RIGHT_BOUNDARY 70

#define FORWARD_MOVING_DIRECTION 90
#define RIGHT_MOVING_DIRECTION 0
#define LEFT_MOVING_DIRECTION 180

#define FORCE_TURN_COUNTER 350


#define FORWARD_SIDE_LIMIT 650
#define RIGHT_SIDE_LIMIT 750
#define LEFT_SIDE_LIMIT RIGHT_SIDE_LIMIT

//Left Hand Algorithm

#define FORWARD_WALL_REACHED_LIMIT 800
#define RIGHT_WALL_REACHED_LIMIT 900
#define LEFT_WALL_REACHED_LIMIT RIGHT_WALL_REACHED_LIMIT

//Grid Navigation
#define PLAN_TIMEOUT 8000 //ms
#define PLAN_MIN_SIZE 20
#define GOAL_REACH_DIST 0.1 //m
#define ANGLE_THRESHOLD (M_PI/30)
#define ROBOT_SPEED (char) 200
#define ROBOT_TURNING_SPEED (char) 120

#endif
