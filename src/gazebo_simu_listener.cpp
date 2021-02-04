/* listener.cpp: read from socket the udp package send by optitrack worstation
 * and publish its structura in a tf message
 */

#include "gazebo_simu_listener.h"

using namespace Eigen;

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
	load_param(model_name, "iris", "model_name");
	load_param(publish_odom_pose, true, "publish_odom_pose");

	/* Topic */
	tfTopic = nodeHandle.advertise<tf::tfMessage>("tf", 0);
	if( publish_odom_pose )
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

	while (!found && model_index < models.name.size() ) {
		if ( models.name[model_index] == model_name ) {
			model_pos = model_index;
			found = true;


		}

		model_index++;
	}


	if( found ) {			
		_obj_pos << models.pose[model_pos].position.x, models.pose[model_pos].position.y, models.pose[model_pos].position.z;
		_obj_quat << models.pose[model_pos].orientation.w, models.pose[model_pos].orientation.x, models.pose[model_pos].orientation.y, models.pose[model_pos].orientation.z;		
		new_obj_data = true;
	}
}


/* read from the socket */
bool listener::Read() {



	ros::Rate r(100);
	geometry_msgs::PoseStamped obj_pose;


	//Matrix3d R_o_ned;
	Matrix4d H_o_ned;
	//Matrix3d R_o_enu;
	Matrix4d H_o_enu;

	H_o_ned << 1, 0, 0, 0,  0, -1, 0, 0,  0, 0, -1, 0, 0, 0, 0, 1;
	H_o_enu << 0, -1, 0, 0,  1, 0, 0, 0,  0, 0, 1, 0, 0, 0, 0, 1;



	obj_pose.header.frame_id = "odom";
	tf::Transform transform;
  	tf::TransformBroadcaster broadcaster;

	//Matrix3d R_opt;
	Matrix4d H_opt = Matrix4d::Identity();
	bool first = true;
	Matrix4d H_opt2_t0 = Matrix4d::Identity();




	while(ros::ok()) {

		obj_pose.header.stamp = ros::Time::now();
		if( new_obj_data ) {

		
			H_opt.block(0,0, 3,3) = utilities::QuatToMat( _obj_quat );
			H_opt(0,3) = _obj_pos(0);
			H_opt(1,3) = _obj_pos(1);
			H_opt(2,3) = _obj_pos(2);
			H_opt(3,3) = 1.0;
			
			if( output_ref_frame == "ned" ) {

				if( first ) {
					first = false;
					H_opt2_t0 = (H_o_ned*H_opt*H_o_ned.transpose()).inverse();
				}

				Matrix4d H_opt2 = H_o_ned*H_opt*H_o_ned.transpose();
				Vector4d q_ned = utilities::MatToQuat(H_opt2.block(0,0,3,3));

				obj_pose.pose.position.x = H_opt2(0,3);
				obj_pose.pose.position.y = H_opt2(1,3);
				obj_pose.pose.position.z = H_opt2(2,3);

				obj_pose.pose.orientation.w = q_ned(0);
				obj_pose.pose.orientation.x = q_ned(1);
				obj_pose.pose.orientation.y = q_ned(2);
				obj_pose.pose.orientation.z = q_ned(3);
			
			}
			
			else if( output_ref_frame == "enu" ) {

				if( first ) {
					first = false;
					H_opt2_t0 = (H_o_enu*H_opt).inverse();
				}

				Matrix4d H_opt2_enu = H_opt2_t0*H_o_enu*H_opt;
				Vector4d q_enu = utilities::MatToQuat(H_opt2_enu.block(0,0,3,3));

				obj_pose.pose.position.x = H_opt2_enu(0,3);
				obj_pose.pose.position.y = H_opt2_enu(1,3);
				obj_pose.pose.position.z = H_opt2_enu(2,3);

				obj_pose.pose.orientation.w = q_enu(0);
				obj_pose.pose.orientation.x = q_enu(1);
				obj_pose.pose.orientation.y = q_enu(2);
				obj_pose.pose.orientation.z = q_enu(3);
		
			}


			if( publish_odom_pose )
				quad_pose.publish( obj_pose );

			//Send tf
			transform.setOrigin(tf::Vector3(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z));
			tf::Quaternion q(obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z, obj_pose.pose.orientation.w);
			transform.setRotation(q);
			tf::StampedTransform stamp_transform(transform, ros::Time::now(), "odom", "base_link");
			broadcaster.sendTransform(stamp_transform);

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
