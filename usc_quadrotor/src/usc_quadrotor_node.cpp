#include "usc_quadrotor.h"
#define UPDATE_RATE 0.1

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
	<< feedback->pose.position.x << ", " << feedback->pose.position.y
	<< ", " << feedback->pose.position.z );
}

class Quadrocopter{	
	InteractiveMarker int_marker;
	ros::Timer timer;
	ros::NodeHandle nh;

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

		tf::pointTFToMsg(position, marker.pose.position);

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

	void move(const ros::TimerEvent& event){
		if(int_marker.pose.position.z > 5)
			int_marker.pose.position.z = 0.0;
	    else
		    int_marker.pose.position.z = int_marker.pose.position.z + 0.1;
	    server->setPose( int_marker.name, int_marker.pose);
		server->applyChanges();
	}
	

public:
	InteractiveMarker get_marker(){
		return int_marker;
	}

	Quadrocopter( 	const tf::Vector3 &position, const std::string &quadrotor_name){
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

		server->insert(int_marker);
  		server->setCallback(int_marker.name, &processFeedback);

		timer = nh.createTimer(ros::Duration(UPDATE_RATE), &Quadrocopter::move, this);
	}
};

bool quadrotor_positions[MAP_SIZE][MAP_SIZE] = {{false}};

tf::Vector3 get_random_pose(){	
	int x = rand()%MAP_SIZE, y = rand()%MAP_SIZE;
	while( quadrotor_positions[x][y]){
		x = rand()%MAP_SIZE;
		y = rand()%MAP_SIZE;
	}

	quadrotor_positions[x][y] = true;
	quadrotor_positions[x-1][y] = true;
	quadrotor_positions[x][y-1] = true;
	quadrotor_positions[x-1][y-1] = true;
	quadrotor_positions[x+1][y+1] = true;
	quadrotor_positions[x+1][y] = true;
	quadrotor_positions[x][y+1] = true;
	quadrotor_positions[x-1][y+1] = true;
	quadrotor_positions[x+1][y-1] = true;

	return tf::Vector3 (x, y, 0.0);;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "usc_quadrotor");
  	
  	// create an interactive marker server on the topic namespace simple_marker
  	server.reset( new interactive_markers::InteractiveMarkerServer("quadrotor_server","",false) );
  	// interactive_markers::InteractiveMarkerServer server("quadrotor_server");

  	Quadrocopter *Q[NUM_OF_QUAD];
	
	for(int i=0; i< NUM_OF_QUAD; i++){
  		char name[5];
  		sprintf(name, "Q%d", (i+1));
  		Q[i] = new Quadrocopter(get_random_pose(), name);
  	}

	server->applyChanges();			// 'commit' changes and send to all clients
	ros::spin();
	server.reset();

	// invoke destructors
	for (int j = 0; j < NUM_OF_QUAD; j++)
	  delete Q[j];
	
	return 0;
}