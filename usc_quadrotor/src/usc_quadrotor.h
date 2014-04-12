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



#include <sstream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

#define FAN_R 0.5
#define MAP_SIZE 50
#define NUM_OF_QUAD 2
#define UPDATE_RATE (1/2)
#define STEP 0.1
#define ALTITUDE 3
#define SLEEP 0.05

#endif