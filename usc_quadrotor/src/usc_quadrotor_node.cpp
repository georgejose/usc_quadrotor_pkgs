#include "usc_quadrotor.h"

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::list<std::vector<double> > source;
std::list<std::vector<double> > destination;

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
	ros::Subscriber sub;
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

	bool gripper(std::string frame, std::string name){
		ros::ServiceClient client = nh.serviceClient<usc_quadrotor::change_frame>("change_frame");
		usc_quadrotor::change_frame srv;
		srv.request.name = name;
		srv.request.frame_id = frame;
		srv.request.x = (int)pos_x;
		srv.request.y = (int)pos_y;
		while(!client.call(srv));

		if(!srv.response.change)
			return false;

		cube_name = srv.response.name; 
		return true;
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
		while( floorf(pos_z*100)/100 > 1.0){
			pos_z -= STEP;
			pos_z = floorf(pos_z*100)/100;
			ros::Duration(SLEEP).sleep();
		}
		pos_z = 1.0;
		ros::Duration(SLEEP).sleep();
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

	bool pick_up(){
		move_down();
		if(!gripper(int_marker.header.frame_id, "none"))
			return false;
		move_up();
		return true;
	}

	int place_block(std::vector<double> s, std::vector<double> d, bool dest){
		
		if(dest)
			goto here;

		ROS_INFO("%s moving block from (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
			int_marker.name.c_str(), s[0],s[1],s[2],d[0],d[1],d[2]);
		move_up();
		move_side(s);
		if(!pick_up())
			return 1;
	here:
		move_side(d);
		move_down();
		if(!gripper("world", cube_name))
			return 2;
		return 0;
	}

	void action(){
		while(!destination.empty()){
			bool dest = false;
			int i,j;
			std::list<std::vector<double> >::iterator it1;
			std::list<std::vector<double> >::iterator it2;

			
			i=rand()%source.size();
			it1 = source.begin();
			while(i--)
				it1++;
		there:
			j=rand()%destination.size();
			it2 = destination.begin();
			while(j--)
				it2++;

			int err = place_block( *it1, *it2, dest);
			
			if(err == 1){ 
				source.erase(it1); 
				continue;
			}
			else if(err == 2){ 
				destination.erase(it2);
				move_up();
				if(destination.empty())
					return;
 				dest = true;
 				goto there;
			}
			
			source.erase(it1); 
			destination.erase(it2);
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
	void subscriber_callback(const std_msgs::String::ConstPtr& msg)
	{	double rotate_angle_xy = 0;
		double rotate_angle_yz = 0;
		double pi=3.14159265;
		double sum_dx = 0;
		double sum_dy = 0;
		double sum_dz = 0;
	 	ROS_INFO("I heard: [%s]", msg->data.c_str());
		//Message Format is: Source_QR, Target_QR, Euclidean Distance, Distance X axis, Distance Y axis.Then repeat.
    		std::stringstream ssin(msg->data.c_str());
    		while (ssin.good()){
			int i = 0;
			std::string arr[6];
			while(i<6){			
				ssin >> arr[i];
				++i;
			}//end while loop
			if(int_marker.name.compare(arr[0]) == 0){
				std::string Qc = arr[1];
				ROS_INFO("I am [%s] colliding with [%s]",int_marker.name.c_str(),Qc.c_str());
				double distance = StringToNumber<double>(arr[2]);
				double dx = StringToNumber<double>(arr[3]);
				double dy = StringToNumber<double>(arr[4]);
				double dz = StringToNumber<double>(arr[5]);
				if(distance <= MIN_DISTANCE || 
				(dx>=-MIN_DISTANCE && dx <= MIN_DISTANCE && 
				dy>=0 && dy <= MIN_DISTANCE+AVOID_SET_LENGHT &&
				dz>=-MIN_DISTANCE && dz <= MIN_DISTANCE)){
					sum_dx += dx;
					sum_dy += dy;
					sum_dz += dz;				
				}//end if			
			}//end if			
    		}//end while loop
		rotate_angle_xy = atan2(sum_dy,sum_dx)+pi/2;
		rotate_angle_yz = atan2(sum_dy,sum_dz)+pi/2;
	}//end subscriber function

	template <typename T>
	T StringToNumber ( const std::string &Text )//Text not by const reference so that the function can be used with a 
	{                               //character array as argument
		std::stringstream ss(Text);
		T result;
		return ss >> result ? result : 0;
	}
	
	
public:
	InteractiveMarker get_marker(){
		return int_marker;
	}
void callback(const std_msgs::StringConstPtr& str){}
	Quadrocopter( const std::vector<double> &position, const std::string &quadrotor_name){

		int_marker.name = quadrotor_name;
		
		pos_x = position[0];
		pos_y = position[1];
		pos_z = position[2];		

		timer = nh.createTimer(ros::Duration(UPDATE_RATE), &Quadrocopter::publisher_callback, this);
		
		sub = nh.subscribe("collision_alert", 10, &Quadrocopter::subscriber_callback,this);

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

void fill(){

	int b= NUM_CUBES;

	for(int i=1; i<MAP_SIZE-1; i++){
		for(int j=1; j<MAP_SIZE-1; j++){
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

	b = NUM_STRUCT;
	for(int i=MAP_SIZE/2; i>0; i--){
		for(int j=MAP_SIZE/2; j>0; j--){
			std::vector<double> v;
			v.push_back((double)i);
			v.push_back((double)j);
			v.push_back((double)0.5);
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

	struct timeval stime; 
	gettimeofday(&stime,NULL);
	srand(stime.tv_usec);

  	server.reset( new interactive_markers::InteractiveMarkerServer("quadrotor_server","",false) );

  	fill();

  	char *name = argv[1];
  	Quadrocopter Q(get_random_pose(), name);

	server->applyChanges();			// 'commit' changes and send to all clients
	ros::spin();
	server.reset();
	
	return 0;
}
