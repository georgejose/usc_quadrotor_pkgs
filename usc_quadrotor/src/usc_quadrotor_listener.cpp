#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "usc_quadrotor.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ros::service::waitForService("get_pose");
  ros::ServiceClient client =
       node.serviceClient<usc_quadrotor::get_pose>("get_pose");
  usc_quadrotor::get_pose srv;
  client.call(srv);

  ros::Publisher collision_pair =
       node.advertise<std_msgs::String>("collision_alert", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  double distance;
  while (node.ok()){
    tf::StampedTransform transform;
    for(int i = 1; i< NUM_OF_QUAD; i++){
       for(int j = i+1; j<= NUM_OF_QUAD; j++){
          try{
	     std::ostringstream s1,s2;
             s1 << "Q" << i;
 	     s2 << "Q" << j;
             listener.lookupTransform(s1.str(), s2.str(), ros::Time(0), transform);
             distance = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
	     if (distance <= AVOIDANCE_DISTANCE){
		std::ostringstream s;
		s << "Q" << i << ", Q" << j << ", D="<<distance << ", dx="<<transform.getOrigin().x()<<", dy="<<transform.getOrigin().y();
		std_msgs::String msg;
		msg.data = s.str();
	    	collision_pair.publish(msg);
		ROS_INFO("Collision between Q%d and Q%d. D=%f, Dx=%f, Dy=%f",i,j,distance,transform.getOrigin().x(),transform.getOrigin().y());
	      }
      	      
	   }
	   catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what());
	   } 
       }
    }
    rate.sleep();
  }
  return 0;
};
