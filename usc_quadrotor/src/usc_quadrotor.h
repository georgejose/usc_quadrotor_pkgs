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
#include "usc_quadrotor/trajectory.h"

#include <sstream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <list>
#include <algorithm>

#include <fstream>

#define FAN_R 0.5
#define MAP_SIZE 50
#define NUM_OF_QUAD 3
#define NUM_CUBES 45
#define NUM_STRUCT 5
#define STEP 0.1

#define ANGLE_STEP 0.01
#define ALTITUDE 2.5
#define UPDATE_RATE 0.01
#define SLEEP 0.05
#define CUBE_SLEEP 0.01


#define AVOIDANCE_DISTANCE 2.0

#define MIN_DISTANCE 2.0
#define AVOID_SET_LENGHT 3.0

#endif
