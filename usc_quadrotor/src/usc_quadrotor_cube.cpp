#include "usc_quadrotor.h"

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::vector<std::vector<double> > source;

void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
	<< feedback->pose.position.x << ", " << feedback->pose.position.y
	<< ", " << feedback->pose.position.z );
}

class Cube{	
	InteractiveMarker int_marker;
	ros::NodeHandle nh;
	ros::Timer timer;
	std::string frame_name;

	double pos_x;
	double pos_y;
	double pos_z;

	Marker makeCube( InteractiveMarker &msg, const tf::Vector3 &position){
		Marker marker;

		marker.type = Marker::CUBE;
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;
		marker.color.r = 0.5;
		marker.color.g = 1.0;
		marker.color.b = 0.5;
		marker.color.a = 1.0;

		tf::pointTFToMsg(position, marker.pose.position);

		return marker;
	}

	InteractiveMarkerControl& makeCubeControl( InteractiveMarker &msg, const tf::Vector3 &position ){
		InteractiveMarkerControl control;
		control.always_visible = true;
		control.markers.push_back( makeCube(msg, tf::Vector3( 0, 0, 0)));
		msg.controls.push_back( control );

		return msg.controls.back();
	}

	void action(){

	}

	void call_action(){
		boost::thread move( &Cube::action, this);
	}

	void publisher_callback(/*const ros::TimerEvent& event*/){
		while(ros::ok()){
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(pos_x, pos_y, pos_z));
			tf::Quaternion q;
			q.setRPY(0, 0, 0);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_name, int_marker.name));
			ros::Duration(CUBE_SLEEP).sleep();
		}
	}
	
public:
	InteractiveMarker get_marker(){
		return int_marker;
	}

	double get_x(){
		return pos_x;
	}
	
	double get_y(){
		return pos_y;
	}

	double get_z(){
		return pos_z;
	}

	void set_x(double x){
		pos_x = x;
	}
	
	void set_y(double y){
		pos_y = y;
	}

	void set_z(double z){
		pos_z = z;
	}

	void set_frame( std::string frame){
		frame_name = frame;
		ROS_INFO("%s's frame is now %s", int_marker.name.c_str(), frame_name.c_str());
	}

	std::string get_name(){
		return int_marker.name;
	}

	Cube( const std::vector<double> &position, const std::string &cube_name){

		int_marker.name = cube_name;
		
		frame_name = "world";

		pos_x = position[0];
		pos_y = position[1];
		pos_z = position[2];		
		
		boost::thread pub(&Cube::publisher_callback, this);
		//timer = nh.createTimer(ros::Duration(UPDATE_RATE), &Cube::publisher_callback, this);

		int_marker.header.frame_id = int_marker.name;
		int_marker.description = cube_name;
		tf::pointTFToMsg(tf::Vector3(0.0, 0.0, 0.0), int_marker.pose.position);
		makeCubeControl(int_marker, tf::Vector3(0.0, 0.0, 0.0));

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
	}
};

void fill(int blocks){
	int b= blocks;
	for(int i=1; i<MAP_SIZE; i++){
		for(int j=1; j<MAP_SIZE; j++){
			std::vector<double> v;
			v.push_back((double)i);
			v.push_back((double)j);
			v.push_back((double)0.5);
			source.push_back(v);
			if(--b<1)
				break;
		}
		if(b<1)
			break;
	}
}

Cube *C[NUM_CUBES];

bool change_frame(usc_quadrotor::change_frame::Request  &req, usc_quadrotor::change_frame::Response &res){
	res.change = false;
	
	if(req.name.compare("none")!=0){
		// ROS_INFO("Trying to drop the block");
		bool exist = false;
		for(int i=0; i<source.size(); i++){
			if(C[i]->get_z() == req.z-0.5 && C[i]->get_x() == req.x && C[i]->get_y() == req.y){
				exist = true;
			}
		}
		if(!exist){
			for(int i=0; i<source.size(); i++){
				if( req.name.compare(C[i]->get_name()) == 0){ 			
		  			C[i]->set_frame(req.frame_id);
					C[i]->set_x((double)req.x);
					C[i]->set_y((double)req.y);
					C[i]->set_z((double)(req.z-0.5));
		  			server->applyChanges();
		  			res.name = "none";
		  			res.change = true;
		  			ROS_INFO("%s's rel_pose is (%.2f %.2f %.2f)", C[i]->get_name().c_str(), C[i]->get_x(), C[i]->get_y(), C[i]->get_z());
		  			break;
		  		}
		  	}
		}
	}
	else{
	  	for(int i=0; i<source.size(); i++){
	  		// ROS_INFO("Looking for block to pick up");
	  		if(C[i]->get_z() == req.z-0.5 && C[i]->get_x() == req.x && C[i]->get_y() == req.y){
	  			C[i]->set_frame(req.frame_id);
  				C[i]->set_x((double)0);
  				C[i]->set_y((double)0);
  				C[i]->set_z((double)-0.5);	
	  			server->applyChanges();
	  			res.name = C[i]->get_name();
	  			res.change = true;
	  			ROS_INFO("%s's rel_pose is (%.2f %.2f %.2f)", C[i]->get_name().c_str(), C[i]->get_x(), C[i]->get_y(), C[i]->get_z());
	  			break;
	  		}
	  	}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "usc_quadrotor");

  	server.reset( new interactive_markers::InteractiveMarkerServer("quadrotor_server","",false) );

  	fill(NUM_CUBES);
	
	for(int i=0; i< source.size(); i++){
  		char name[5];
  		sprintf(name, "C%d", (i+1));
  		C[i] = new Cube(source[i], name);
  	}

	server->applyChanges();			// 'commit' changes and send to all clients
	
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("change_frame", change_frame);

	ros::spin();
	server.reset();
	
	return 0;
}
