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
    std::ostringstream s;
    s<<"";
    for(int i = 1; i<= NUM_OF_QUAD; i++){
       for(int j = 1; j<= NUM_OF_QUAD; j++){
	  if(i != j){
		  try{
			std::ostringstream s1,s2;
			s1 << "Q" << i; //Source frame
		 	s2 << "Q" << j; // Target frame
			listener.lookupTransform(s2.str(), s1.str(), ros::Time(0), transform);
			distance = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
			double dx = transform.getOrigin().x();
			double dy = transform.getOrigin().y();
			if(distance <= MIN_DISTANCE || (dx>=-MIN_DISTANCE && dx <= MIN_DISTANCE && dy>=0 && dy <= MIN_DISTANCE+AVOID_SET_LENGHT)){
				s <<"Q"<< i << " Q" << j << " "<<distance << " "<<dx<<" "<<dy<< " ";
			}//end if
		   }//end try
		   catch (tf::TransformException ex){
		      	ROS_ERROR("%s",ex.what());
		   }//end catch 
	   }//end if
       }//end loop
    }//end loop
    if(s.str().compare("") != 0){
	    std_msgs::String msg;
	    msg.data = s.str();
	    //Message Format is: Source_QR, Target_QR, Euclidean Distance, Distance X axis, Distance Y axis.Then repeat.
	    collision_pair.publish(msg);
	    //ROS_INFO("Collision between Q%d and Q%d. D=%f, Dx=%f, Dy=%f",i,j,distance,transform.getOrigin().x(),transform.getOrigin().y());
    }//end if
    rate.sleep();
  }
  return 0;
};
