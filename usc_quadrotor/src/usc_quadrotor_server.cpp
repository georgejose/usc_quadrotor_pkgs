#include "usc_quadrotor.h"

bool quadrotor_positions[MAP_SIZE][MAP_SIZE][MAP_SIZE] = {{false}};

bool check_map(int x, int y, int z){
  return quadrotor_positions[x][y][z];
}


bool get_position(usc_quadrotor::get_pose::Request  &req, usc_quadrotor::get_pose::Response &res){

  if(req.goal){
    res.selected = check_map(req.x, req.y, req.z) ? false : true;
    if(res.selected)
      quadrotor_positions[req.x][req.y][req.z] = true;
  }
  else{
    int x,y,z;
    do{
      x = rand()%MAP_SIZE;
      y = rand()%MAP_SIZE;
    }while( check_map(x, y, MAP_SIZE/2));
    
    quadrotor_positions[x][y][MAP_SIZE/2] = true;

    res.x = x;
    res.y = y;
    res.z = MAP_SIZE/2; 
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "add_two_ints_server");
  
  struct timeval stime; 
  gettimeofday(&stime,NULL);
  srand((stime.tv_sec * 1000) + (stime.tv_usec / 1000));

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("get_pose", get_position);
  
  ROS_INFO("Pose Server is ready");
  
  ros::spin();

  return 0;
}