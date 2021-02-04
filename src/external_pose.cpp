/* listener.cpp: read from socket the udp package send by optitrack worstation
 * and publish its structura in a tf message
 */

#include "gazebo_simu_listener.h"

using namespace TooN;

void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

listener::listener() {

	/* node params */
	load_param(port, 9030, "port");
	load_param(blocking, false, "blocking");
	load_param(debug, false, "debug");
	load_param(output_topic_name, "/mavros/mocap/pose", "output_topic_name");
	load_param(output_ref_frame, "ned", "output_ref_frame");
	load_param(model_name, "iris_with_camera", "model_name");

	/* Topic */
	tfTopic = nodeHandle.advertise<tf::tfMessage>("tf", 0);
	quad_pose = nodeHandle.advertise<geometry_msgs::PoseStamped>(output_topic_name.c_str(), 0);
	ext_pos_sub = nodeHandle.subscribe("/gazebo/model_states", 0, &listener::gazebo_models_cb, this);


	new_obj_data = false;
}

listener::~listener() {
	//TODO: close socket

}

void listener::gazebo_models_cb( gazebo_msgs::ModelStates models ) {

	bool found = false;

	int model_index = 0;
	int model_pos = -1;

	while (!found || model_index < models.name.size() ) {
		if ( models.name[model_index] == model_name ) {
			model_pos = model_index;
			found = true;


		}

		model_index++;
	}

	if( found ) {

			obj_pos = makeVector( models.pose[model_pos].position.x, models.pose[model_pos].position.y, models.pose[model_pos].position.z);
			obj_quat = makeVector( models.pose[model_pos].orientation.w, models.pose[model_pos].orientation.x, models.pose[model_pos].orientation.y, models.pose[model_pos].orientation.z);
			new_obj_data = true;

/*
			Matrix<3> R_go;
			Fill(R_go) = 0, 1, 0,
									 0, 0, 1,
									 1, 0, 0;

		Matrix<3> R_g = robohelper::QuatToMat(obj_quat_g);
		obj_pos = R_go*obj_pos_g;
		Matrix<3> R_opt2 = R_go*R_g*R_go.T();
		obj_quat = robohelper::MatToQuat(R_opt2);

		new_obj_data = true;
		*/

	}
}


/* read from the socket */
bool listener::Read() {



	ros::Rate r(100);
	geometry_msgs::PoseStamped obj_pose;

	Matrix<3> R_o_ned;
	Fill(R_o_ned) =  1,  0, 0,
									0,  -1, 0,
									 0, 0, -1;
	Matrix<3> R_o_enu;
	Fill(R_o_enu) =  0,  -1, 0,
									 1,  0, 0,
									 0,  0, 1;


	obj_pose.header.frame_id = "map";


	while(ros::ok()) {
		obj_pose.header.stamp = ros::Time::now();
		if( new_obj_data ) {

			Matrix<3> R_opt = robohelper::QuatToMat(obj_quat);


			if( output_ref_frame == "ned" ) {

				Vector<3> p_ned = R_o_ned*obj_pos;
				Matrix<3> R_opt2 = R_o_ned*R_opt*R_o_ned.T();
				Vector<4> q_ned = robohelper::MatToQuat(R_opt2);

				obj_pose.pose.position.x = p_ned[0];
				obj_pose.pose.position.y = p_ned[1];
				obj_pose.pose.position.z = p_ned[2];

				obj_pose.pose.orientation.w = q_ned[0];
				obj_pose.pose.orientation.x = q_ned[1];
				obj_pose.pose.orientation.y = q_ned[2];
				obj_pose.pose.orientation.z = q_ned[3];
			}

			else if( output_ref_frame == "enu" ) {

				Vector<3> p_enu = R_o_enu*obj_pos;


				Matrix<3> R_opt_enu2 = R_o_enu*R_opt;
				Vector<4> q_enu = robohelper::MatToQuat(R_opt_enu2);

				obj_pose.pose.position.x = p_enu[0];
				obj_pose.pose.position.y = p_enu[1];
				obj_pose.pose.position.z = p_enu[2];

				obj_pose.pose.orientation.w = q_enu[0];
				obj_pose.pose.orientation.x = q_enu[1];
				obj_pose.pose.orientation.y = q_enu[2];
				obj_pose.pose.orientation.z = q_enu[3];
			}



			quad_pose.publish( obj_pose );

			new_obj_data = false;
		}


		r.sleep();
	}
}



int listener::run() {
	boost::thread read_t( &listener::Read, this );
	ros::spin();
}

/* main */
int main(int argc, char** argv) {
	ros::init(argc, argv, "optitrack_listener");
	listener listener_;
	listener_.run();
	return 0;
}
