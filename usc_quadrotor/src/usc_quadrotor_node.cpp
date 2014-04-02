#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <string.h>

#define FAN_R 0.5
using namespace visualization_msgs;

void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback ){
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
	<< feedback->pose.position.x << ", " << feedback->pose.position.y
	<< ", " << feedback->pose.position.z );
}

class Quadrocopter{
	
	InteractiveMarker int_marker;

	Marker makeCylinder( InteractiveMarker &msg, const tf::Vector3 &position){
		Marker marker;

		marker.type = Marker::CYLINDER;
		marker.scale.x = FAN_R;
		marker.scale.y = FAN_R;
		marker.scale.z = 0.1;
		marker.color.r = 0.5;
		marker.color.g = 0.5;
		marker.color.b = 0.5;
		marker.color.a = 1.0;

		marker.pose.position.x = position.m_floats[0];
		marker.pose.position.y = position.m_floats[1];
		// marker.pose.position.z = position.m_floats[2];
		// tf::pointTFToMsg(position, marker.pose.position);

		return marker;
	}

	InteractiveMarkerControl& makeCylinderControl( InteractiveMarker &msg, const tf::Vector3 &position ){
		InteractiveMarkerControl control;
		control.always_visible = true;
		float d = FAN_R / 1.414;
		control.markers.push_back( makeCylinder(msg, tf::Vector3( - d, - d, 0)));
		control.markers.push_back( makeCylinder(msg, tf::Vector3( - d, + d, 0)));
		control.markers.push_back( makeCylinder(msg, tf::Vector3( + d, + d, 0)));
		control.markers.push_back( makeCylinder(msg, tf::Vector3( + d, - d, 0)));
		msg.controls.push_back( control );

		return msg.controls.back();
	}

public:

	InteractiveMarker get_marker(){
		return int_marker;
	}

	Quadrocopter( const tf::Vector3 &position, const std::string &quadrotor_name){
		int_marker.header.frame_id = "base_link";
		tf::pointTFToMsg(position, int_marker.pose.position);

		int_marker.name = quadrotor_name;
		int_marker.description = quadrotor_name;

		makeCylinderControl(int_marker, position);

		InteractiveMarkerControl control;

		control.orientation.w = 1;
		control.orientation.x = 0;
		control.orientation.y = 1;
		control.orientation.z = 0;
		control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
		int_marker.controls.push_back(control);
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "usc_quadrotor");

  	// create an interactive marker server on the topic namespace simple_marker
  	interactive_markers::InteractiveMarkerServer server("quadrotor_server");

 	Quadrocopter Q1(tf::Vector3(0.0, 0.0, 0.0), "Q1");
	server.insert( Q1.get_marker(), &processFeedback);	// tell the server to call processFeedback() when feedback arrives for it
	
	Quadrocopter Q2(tf::Vector3(2.0, 3.0, 4.0), "Q2");
	server.insert( Q2.get_marker(), &processFeedback);

	Quadrocopter Q3(tf::Vector3(3.0, 3.0, 3.0), "Q3");
	server.insert( Q3.get_marker(), &processFeedback);
	server.applyChanges();			// 'commit' changes and send to all clients

	// start the ROS main loop
	ros::spin();
	return 0;
}