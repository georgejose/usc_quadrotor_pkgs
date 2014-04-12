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
	ros::NodeHandle nh;
	ros::Timer timer;
	std::string cube_name;

	double pos_x;
	double pos_y;
	double pos_z;

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

	void gripper(std::string frame, std::string name){
		ros::ServiceClient client = nh.serviceClient<usc_quadrotor::change_frame>("change_frame");
		usc_quadrotor::change_frame srv;
		srv.request.name = name;
		srv.request.frame_id = frame;
		srv.request.x = (int)pos_x;
		srv.request.y = (int)pos_y;
		while(!client.call(srv));

		cube_name = srv.response.name; 
	}

	void move_up(){
		while( floorf(pos_z*100)/100 < ALTITUDE ){
			pos_z += STEP;
			pos_z = floorf(pos_z*100)/100;
			ros::Duration(SLEEP).sleep();
		}
		pos_z = ALTITUDE;
		ros::Duration(SLEEP).sleep();
	}
	
	void move_down(){
		while( floorf(pos_z*100)/100 > 0.0){
			pos_z -= STEP;
			pos_z = floorf(pos_z*100)/100;
			ros::Duration(SLEEP).sleep();
		}
		pos_z = 0.0;
		ros::Duration(SLEEP).sleep();
	}

	void pick_up(){
		move_down();
		gripper(int_marker.header.frame_id, "none");
		move_up();
	}

	void move_side(std::vector<double> d){

		while(	floorf((d[0]-pos_x)*10)/10 ||
				floorf((d[1]-pos_y)*10)/10){

			if( floorf((d[0]-pos_x)*10)/10){
				pos_x += (d[0] - pos_x > 0 ? STEP : -STEP);
				pos_x = floorf(pos_x*10)/10;
			}

			if( floorf((d[1]-pos_y)*10)/10){
				pos_y += (d[1] - pos_y > 0 ? STEP : -STEP);
				pos_y = floorf(pos_y*10)/10;
			}
			ros::Duration(SLEEP).sleep();
		}
			pos_x = d[0];
			pos_y = d[1];
			ros::Duration(SLEEP).sleep();
	}

	void place_block(int i, int j){
		ROS_INFO("%s moving block from (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
			int_marker.name.c_str(), source[i][0],source[i][1],source[i][2],
			destination[j][0],destination[j][1],destination[j][2]);
		move_up();
		move_side(source[i]);
		pick_up();
		move_side(destination[j]);
		move_down();
		gripper("world", cube_name);
	}

	void action(){
		while(!destination.empty()){
			int i,j;
			do{
				i=rand()%source.size();
			}while(source[i][0]==-1);

			do{
				j=rand()%destination.size();
			}while(destination[i][0]==-1);

			place_block( i, j);
			source[i][0]=-1;
			destination[j][0]=-1;
		}
	}

	void call_action(){
		boost::thread move( &Quadrocopter::action, this);
	}

	void publisher_callback(const ros::TimerEvent& event){
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(pos_x, pos_y, pos_z));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", int_marker.name));
	}
	
public:
	InteractiveMarker get_marker(){
		return int_marker;
	}

	Quadrocopter( const std::vector<double> &position, const std::string &quadrotor_name){

		int_marker.name = quadrotor_name;
		
		pos_x = position[0];
		pos_y = position[1];
		pos_z = position[2];		

		timer = nh.createTimer(ros::Duration(UPDATE_RATE), &Quadrocopter::publisher_callback, this);
		
		int_marker.header.frame_id = int_marker.name;
		int_marker.description = quadrotor_name;
		tf::pointTFToMsg(tf::Vector3(0.0, 0.0, 0.0), int_marker.pose.position);
		makeCylinderControl(int_marker, tf::Vector3(0.0, 0.0, 0.0));

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

  		call_action();
	}
};

std::vector<double> get_random_pose(){	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<usc_quadrotor::get_pose>("get_pose");
	usc_quadrotor::get_pose srv;
	std::vector<double> v;

	while(!client.call(srv));

	ROS_INFO("Position for is (%d %d %d)",
		srv.response.x, srv.response.y, srv.response.z);
	
	v.push_back((double)srv.response.x);
	v.push_back((double)srv.response.y);
	v.push_back((double)srv.response.z);
	return v;
}

void fill(int blocks){

	int b= blocks;

	for(int i=0; i<MAP_SIZE; i++){
		for(int j=0; j<MAP_SIZE; j++){
			std::vector<double> v;
			v.push_back((double)i);
			v.push_back((double)j);
			v.push_back((double)0);
			source.push_back(v);
			if(--b<1)
				break;
		}
		if(b<1)
			break;
	}

	b = blocks;
	for(int i=MAP_SIZE-1; i>=0; i--){
		for(int j=MAP_SIZE-1; j>=0; j--){
			std::vector<double> v;
			v.push_back((double)i);
			v.push_back((double)j);
			v.push_back((double)0);
			destination.push_back(v);
			if(--b<1)
				break;
		}
		if(b<1)
			break;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "usc_quadrotor");
  		
  	server.reset( new interactive_markers::InteractiveMarkerServer("quadrotor_server","",false) );

  	char *name = argv[1];
	
  	fill(5);

  	Quadrocopter Q(get_random_pose(), name);

	server->applyChanges();			// 'commit' changes and send to all clients
	ros::spin();
	server.reset();
	
	return 0;
}
