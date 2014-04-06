#include "usc_quadrotor.h"

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::vector<std::vector<double> > source;
std::vector<std::vector<double> > destination;

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

	void move_up(){
		while( floorf(int_marker.pose.position.z*100)/100 < ALTITUDE ){
			int_marker.pose.position.z += STEP;
			int_marker.pose.position.z = floorf(int_marker.pose.position.z*100)/100;
		    server->setPose( int_marker.name, int_marker.pose);
			server->applyChanges();
			ros::Duration(SLEEP).sleep();
		}
		int_marker.pose.position.z = ALTITUDE;
	    server->setPose( int_marker.name, int_marker.pose);
		server->applyChanges();
		ros::Duration(SLEEP).sleep();
	}
	
	void move_down(){
		while( floorf(int_marker.pose.position.z*100)/100 > 0.0){
			int_marker.pose.position.z -= STEP;
			int_marker.pose.position.z = floorf(int_marker.pose.position.z*100)/100;
		    server->setPose( int_marker.name, int_marker.pose);
			server->applyChanges();
			ros::Duration(SLEEP).sleep();
		}
		int_marker.pose.position.z = 0.0;
	    server->setPose( int_marker.name, int_marker.pose);
		server->applyChanges();
		ros::Duration(SLEEP).sleep();
	}

	void pick_up(){
		move_down();
		move_up();
	}

	void move_side(std::vector<double> d){
		
		ROS_INFO_STREAM(d[0]);
		ROS_INFO_STREAM(d[1]);
		
		while(	floorf((d[0]-int_marker.pose.position.x)*10)/10 ||
				floorf((d[1]-int_marker.pose.position.y)*10)/10){

			// ROS_INFO_STREAM( floorf((d[1]-int_marker.pose.position.y)*100)/100 );

			if( floorf((d[0]-int_marker.pose.position.x)*10)/10){
				int_marker.pose.position.x += (d[0] - int_marker.pose.position.x > 0 ? STEP : -STEP);
				int_marker.pose.position.x = floorf(int_marker.pose.position.x*10)/10;
			}

			if( floorf((d[1]-int_marker.pose.position.y)*10)/10){
				int_marker.pose.position.y += (d[1] - int_marker.pose.position.y > 0 ? STEP : -STEP);
				int_marker.pose.position.y = floorf(int_marker.pose.position.y*10)/10;
			}

		    server->setPose( int_marker.name, int_marker.pose);
			server->applyChanges();
			ros::Duration(SLEEP).sleep();
		}
			int_marker.pose.position.y = d[0];
			int_marker.pose.position.y = d[1];
			server->setPose( int_marker.name, int_marker.pose);
			server->applyChanges();
			ros::Duration(SLEEP).sleep();
	}

	void place_block(std::vector<double> s, std::vector<double> d){
		move_up();
		move_side(s);
		pick_up();
		move_side(d);
		move_down();
	}

	void action(const ros::TimerEvent& event){
		while(!destination.empty()){
			place_block(source.back(), destination.back());
			source.pop_back();
			destination.pop_back();
		}
		// ROS_INFO_STREAM(int_marker.name);
		// ros::Duration(3).sleep();
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
  		server->applyChanges();
		timer = nh.createTimer(ros::Duration(UPDATE_RATE), &Quadrocopter::action, this, true);
	}
};

bool quadrotor_positions[MAP_SIZE][MAP_SIZE] = {{false}};

bool check_map(int x, int y){
	return 	quadrotor_positions[x  ][y  ] ||
			quadrotor_positions[x-1][y  ] ||
			quadrotor_positions[x  ][y-1] ||
			quadrotor_positions[x-1][y-1] ||
			quadrotor_positions[x+1][y+1] ||
			quadrotor_positions[x+1][y  ] ||
			quadrotor_positions[x  ][y+1] ||
			quadrotor_positions[x-1][y+1] ||
			quadrotor_positions[x+1][y-1];
}

tf::Vector3 get_random_pose(){	
	int x,y;
	do{
		x = rand()%MAP_SIZE;
		y = rand()%MAP_SIZE;
	}while( check_map(x,y));
	
	quadrotor_positions[x][y] = true;
	return tf::Vector3 (x, y, 0.0);
}

std::vector<double> get_block(){	
	int x,y;
	do{
		x = rand()%MAP_SIZE;
		y = rand()%MAP_SIZE;
	}while( check_map(x,y));
	
	quadrotor_positions[x][y] = true;
	
	std::vector<double> d;
	d.push_back((double)x);
	d.push_back((double)y);
	d.push_back((double)0);
	return d;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "usc_quadrotor");
  		
  	server.reset( new interactive_markers::InteractiveMarkerServer("quadrotor_server","",false) );
  	
  	srand ((unsigned int)time(NULL));
	 
  	source.push_back(get_block());
  	source.push_back(get_block());

  	destination.push_back(get_block());
  	destination.push_back(get_block());

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