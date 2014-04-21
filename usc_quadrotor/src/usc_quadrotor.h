#ifndef __X_H_INCLUDED__
#define __X_H_INCLUDED__
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include "std_msgs/String.h"
#include <boost/thread/mutex.hpp>

#include "usc_quadrotor/change_frame.h"
#include "usc_quadrotor/get_pose.h"

#include <sstream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <list>
#include <algorithm>

#define FAN_R 0.5
#define MAP_SIZE 50
#define NUM_OF_QUAD 2
#define NUM_CUBES 15
#define CUBE_SLEEP 0.01
#define NUM_STRUCT 5
/*<<<<<<< HEAD
#define UPDATE_RATE 0.1
=======*/
#define UPDATE_RATE (1/2)
#define CUBE_SLEEP 0.01
// >>>>>>> b3c32f042586516559292f1e43a1e90674433be2
#define STEP 0.1
#define ALTITUDE 3
#define SLEEP 0.05
#define AVOIDANCE_DISTANCE 5.0

#define MIN_DISTANCE 3.0
#define AVOID_SET_LENGHT 2.0


#endif
