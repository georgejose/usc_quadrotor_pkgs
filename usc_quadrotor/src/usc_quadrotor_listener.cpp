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
			listener.lookupTransform(s1.str(), s2.str(), ros::Time(0), transform);
			double dx = transform.getOrigin().x();
			double dy = transform.getOrigin().y();
			double dz = transform.getOrigin().z();
			
			distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
			
			if(distance <= MIN_DISTANCE || 
			(dy>=-MIN_DISTANCE && dy <= MIN_DISTANCE && 
			dx>=0 && dx <= MIN_DISTANCE+AVOID_SET_LENGHT &&
			dz>=-MIN_DISTANCE && dz <= MIN_DISTANCE)){
				s <<"Q"<< i << " Q" << j << " "<<distance << " "<<dx<<" "<<dy<< " "<<dz<< " ";
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
	    //Message Format is: Source_QR, Target_QR, Euclidean Distance, Distance X axis, Distance Y axis, Distance Z axis.Then repeat.
	    collision_pair.publish(msg);
	    //ROS_INFO("Collision between Q%d and Q%d. D=%f, Dx=%f, Dy=%f, Dz=%f",i,j,distance,transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    }//end if
    rate.sleep();
  }//end of while loop
  return 0;
};
