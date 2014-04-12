#include "ros/ros.h"
#include "usc_quadrotor.h"
#include "usc_quadrotor/get_pose.h"

bool quadrotor_positions[MAP_SIZE][MAP_SIZE] = {{false}};

bool check_map(int x, int y){
  return  quadrotor_positions[x  ][y  ] ||
          quadrotor_positions[x-1][y  ] ||
          quadrotor_positions[x  ][y-1] ||
          quadrotor_positions[x-1][y-1] ||
          quadrotor_positions[x+1][y+1] ||
          quadrotor_positions[x+1][y  ] ||
          quadrotor_positions[x  ][y+1] ||
          quadrotor_positions[x-1][y+1] ||
          quadrotor_positions[x+1][y-1];
}


bool get_position(usc_quadrotor::get_pose::Request  &req, usc_quadrotor::get_pose::Response &res){
  
  int x,y;
  do{
    x = rand()%MAP_SIZE;
    y = rand()%MAP_SIZE;
  }while( check_map(x,y));
  
  quadrotor_positions[x][y] = true;

  res.x = x;
  res.y = y;
  res.z = 0; 
}

int main(int argc, char **argv){
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  srand ((unsigned int)time(NULL));
  
  ros::ServiceServer service = n.advertiseService("get_pose", get_position);
  ROS_INFO("Position generated");
  ros::spin();

  return 0;
}