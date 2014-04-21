#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "usc_quadrotor.h"
#include "usc_quadrotor/get_pose.h"

std::string quadrotor_name;



void poseCallback(const usc_quadrotor::get_pose::Response &res){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(res.x, res.y, res.z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", quadrotor_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need quadrotor name as argument"); return -1;};
  quadrotor_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(quadrotor_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};


