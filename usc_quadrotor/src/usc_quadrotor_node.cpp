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
	int marker_id;
	ros::NodeHandle nh;
	ros::Timer timer;
	ros::Subscriber sub;
	std::string cube_name;

	bool colliding;
	bool slave;
	std::string colliding_with;

	double pos_x;
	double pos_y;
	double pos_z;
	double roll;
	double pitch;
	double yaw;

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
		double d = FAN_R / 2.0;
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
		srv.request.z = (int)pos_z;
		
		while(!client.call(srv));

		if(!srv.response.change)
			return false;

		cube_name = srv.response.name; 
		return true;
	}

	std::vector<int> get_plan(std::vector<double> d, int edit){	
		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<usc_quadrotor::trajectory>("get_plan");
		usc_quadrotor::trajectory srv;

		srv.request.edit = edit;
		
		srv.request.source.push_back(pos_x);
		srv.request.source.push_back(pos_y);
		srv.request.source.push_back(pos_z);

		srv.request.destination.push_back((int)d[0]);
		srv.request.destination.push_back((int)d[1]);
		srv.request.destination.push_back((int)d[2]);

		while(!client.call(srv));

		return srv.response.path;
	}

	bool check_goal(std::vector<double> g){	
		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<usc_quadrotor::get_pose>("get_pose");
		usc_quadrotor::get_pose srv;
		std::vector<double> v;

		srv.request.goal = true;
		srv.request.x = (int)g[0];
		srv.request.y = (int)g[1];
		srv.request.z = (int)g[2];
		
		while(!client.call(srv));

		return srv.response.selected;
	}

	void move_up(){

		double p = pos_z + ALTITUDE + 0.5;
		
		while( floorf(pos_z*100)/100 < p ){
			pos_z += STEP;
			pos_z = floorf(pos_z*100)/100;
			ros::Duration(SLEEP).sleep();
		}
		pos_z = p;
		ros::Duration(SLEEP).sleep();
	}
	
	void move_down(){
		set_orientation(pos_x, pos_y, pos_z);

		double p = pos_z - ALTITUDE + 0.5;

		while( floorf(pos_z*100)/100 > p){
			pos_z -= STEP;
			pos_z = floorf(pos_z*100)/100;
			ros::Duration(SLEEP).sleep();
		}
		pos_z = p;
		ros::Duration(SLEEP).sleep();
	}

	bool pick_up(){
		move_down();
		if(!gripper(int_marker.header.frame_id, "none"))
			return false;
		move_up();
		return true;
	}

	void move_to(double x, double y, double z){
		while(	floorf((x-pos_x)*10)/10 ||
				floorf((y-pos_y)*10)/10 ||
				floorf((z-pos_z)*10)/10 ){

			// ROS_INFO("curently at %f %f %f", pos_x , pos_y , pos_z);

			if( floorf((x-pos_x)*10)/10){
				pos_x += (x - pos_x > 0 ? STEP : -STEP);
				pos_x = floorf(pos_x*10)/10;
			}

			if( floorf((y-pos_y)*10)/10){
				pos_y += (y - pos_y > 0 ? STEP : -STEP);
				pos_y = floorf(pos_y*10)/10;
			}

			if( floorf((z-pos_z)*10)/10){
				pos_z += (z - pos_z > 0 ? STEP : -STEP);
				pos_z = floorf(pos_z*10)/10;
			}
			ros::Duration(SLEEP).sleep();
		}
			pos_x = x;
			pos_y = y;
			pos_z = z;
			ros::Duration(SLEEP).sleep();
	}

	void set_orientation(double x, double y, double z){

			double angle = yaw;
			
			if(x!=pos_x){
				if(y==pos_y && x<pos_x)
					angle = 3.14;
				else
					angle = (y-pos_y)/(x-pos_x);
				
				if(abs(angle)==1.0 && x<pos_x)
					angle *= -2.35;
			}
			else if(y>pos_y)
				angle = 1.57; /* pi/2 */
			else
				angle = -1.57; /* pi/2 */
			
			// ROS_INFO("%f", angle);

/*			while((int)((yaw-angle)*100)){
				yaw+= ( yaw > angle ? -ANGLE_STEP : ANGLE_STEP);
				// yaw = angle;
				ros::Duration(SLEEP/10).sleep();
			}*/
			yaw = angle;

			angle = 0.0;
			if(z!=pos_z)
				angle = z > pos_z ? -0.78 : 0.78; /* pi/4 */
			
			while((int)((pitch-angle)*100)){
				pitch+= ( pitch > angle ? -ANGLE_STEP : ANGLE_STEP);
				ros::Duration(SLEEP/10).sleep();
			}
			pitch = angle;
			
	}

	std::vector<double> get_delta(int v){
		double dX = (double)(v/100) != 2 ? (double)(v/100) : - 1;
		double dY = (double)((v%100)/10) !=2 ? (double)((v%100)/10) : - 1;
		double dZ = (double)(v%10) !=2 ? (double)(v%10) : - 1;

		std::vector<double> x;
		x.push_back(dX);
		x.push_back(dY);
		x.push_back(dZ);

		return x;
	}

	void fly_to(std::vector<double> d){
		
		std::vector<int> v(get_plan(d, 0));

		// ROS_INFO("Received Plan from planner - size %d", (int)v.size());

		for(int i=1; i < v.size(); i++){
			
			std::vector<double> delta(get_delta(v[i]));

			set_orientation( pos_x + delta[0], pos_y + delta[1], pos_z + delta[2]);
			
			if(colliding){
				if(resolve_collision( d) == 1)
					return;
			}
			
			move_to( pos_x + delta[0], pos_y + delta[1], pos_z + delta[2]);
		}
	}

	int resolve_collision(std::vector<double> d){
		if(slave){
			ROS_INFO("colliding %s SLAVE", int_marker.name.c_str());
			std::vector<double> t;
			t.push_back((double)pos_x);
			t.push_back((double)pos_y);
			t.push_back((double)pos_z);

			get_plan(t ,1);
			
			ros::Duration(4).sleep();

			get_plan(t ,2);

			colliding= false;
			colliding_with = "";
			slave = false;
			return 0;
		}
		else{
			ROS_INFO("colliding %s MASTER", int_marker.name.c_str());
			ros::Duration(1).sleep();
			colliding= false;
			fly_to(d);		
			colliding_with = "";	
			return 1;
		}
	}

	int place_block(std::vector<double> s, std::vector<double> d, bool dest){
		
		if(dest)
			goto here;

		ROS_INFO("%s moving block from (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
			int_marker.name.c_str(), s[0],s[1],s[2],d[0],d[1],d[2]);
		move_up();
		
		s[2] += ALTITUDE;
		fly_to(s);
		
		if(!pick_up())
			return 1;
	here:
		d[2] += ALTITUDE;
		fly_to(d);

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
			std::list<std::vector<double> >::iterator it3;

			
			i=rand()%source.size();
			it1 = source.begin();
			while(i--)
				it1++;

			if(!check_goal(*it1)){
				destination.erase(it1);
				continue;
			}

		there:
			j=rand()%destination.size();
			it2 = destination.begin();
			while(j--)
				it2++;

			it3 = destination.begin();
			while(it3!= destination.end()){
				if( (*it3)[0]==(*it2)[0] && 
					(*it3)[1]==(*it2)[1] && 
					(*it3)[2]< (*it2)[2])
					it2 = it3;
				it3++;
			}

			if(!check_goal(*it2)){
				destination.erase(it2);
				if(destination.empty())
					return;
				goto there;
			}

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
			get_plan(*it2, 1);
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
		q.setRPY(roll, pitch, yaw);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", int_marker.name));
	}

	void subscriber_callback(const std_msgs::String::ConstPtr& msg){	
	 	// ROS_INFO("I heard: [%s]", msg->data.c_str());
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
					if(Qc.compare(colliding_with) != 0 && !slave){
					// if(colliding_with.compare("")==0){
						colliding_with = Qc;
						// ROS_ERROR("I heard: [%s]", msg->data.c_str());
						ROS_ERROR("I am Q[%d] colliding with [%s]",marker_id,Qc.c_str());

						int b = StringToNumber<int>(arr[1].substr(1,2));
						
						// ROS_ERROR("%d %f %d", marker_id, StringToNumber<double>(arr[2]), b);
						
						if(StringToNumber<double>(arr[2]) < MIN_DISTANCE)
							slave = true;	
						else if( marker_id < b)
							slave = true;

						colliding = true;
					
					}//end if		
				}//end if			
    		}//end while loop
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

	Quadrocopter( const std::vector<double> &position, const std::string &quadrotor_name){

		int_marker.name = quadrotor_name;
		marker_id = StringToNumber<double>(quadrotor_name.substr(1,2));
		pos_x = position[0];
		pos_y = position[1];
		pos_z = position[2];
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;	

		colliding = false;
		slave = false;
		//moving_to_tempPoint = false;
		colliding_with = "";

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

	srv.request.goal = false;
	while(!client.call(srv));

	// ROS_INFO("Random position (%d %d %d)", srv.response.x, srv.response.y, srv.response.z);
	
	v.push_back((double)srv.response.x);
	v.push_back((double)srv.response.y);
	v.push_back((double)srv.response.z);
	return v;
}

void fill( char* file){

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

	std::ifstream i;
	i.open(file);
	
	char *temp = new char[3];
	
	while(i>>temp){
		std::vector<double> v;
		v.push_back(atof(temp));
		for(int j=0; j<2; j++){
			i>>temp;
			v.push_back(atof(temp));
		}
		destination.push_back(v);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "usc_quadrotor");

	struct timeval stime; 
	gettimeofday(&stime,NULL);
	srand(stime.tv_usec);

  	server.reset( new interactive_markers::InteractiveMarkerServer("quadrotor_server","",false) );

  	fill(argv[2]);

  	char *name = argv[1];
  	Quadrocopter Q(get_random_pose(), name);

	server->applyChanges();			// 'commit' changes and send to all clients

	ros::spin();
	server.reset();
	
	return 0;
}
