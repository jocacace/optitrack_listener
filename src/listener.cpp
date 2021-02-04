
/* listener.cpp: read from socket the udp package send by optitrack worstation
 * and publish its structura in a tf message
 */

#include "listener.h"

//using namespace TooN;
using namespace Eigen;
using namespace std;


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


	/* Topic */
	tfTopic = nodeHandle.advertise<tf::tfMessage>("tf", 0);
	quad_pose = nodeHandle.advertise<geometry_msgs::PoseStamped>(output_topic_name.c_str(), 0);
}

listener::~listener() {
	//TODO: close socket

}

/* open socket */
bool listener::Open(int port_number) {

	if ( (opt_socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		std::cout << "Listener::Open: error during socket creation!" << std::endl;
		return false;
	}
	else
		std::cout << "Listener::Open completed!" << std::endl;

	/* blocking or no-blocking */
	if (blocking)
		fcntl(opt_socket,F_SETFL,O_NONBLOCK);

	memset((char *) &si_me, 0, sizeof(si_me));

	/* allow connections to any address port */
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port_number);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(opt_socket, (struct sockaddr*)&si_me, sizeof(si_me))==-1) {
		std::cout << "Listener::Open: error during bind!" << std::endl;
		return false;
	}
	return true;
}

/* read from the socket */
bool listener::Read() {


	geometry_msgs::PoseStamped pose;
	/* clear output message */
	tfPublish.transforms.clear();
	slen=sizeof(si_other);
	rlen = recvfrom(opt_socket, buffer, 1024, 0,(struct sockaddr*)&si_other, (socklen_t*)&slen);
	char pck_type = (char)buffer[0];

	if (pck_type == 'O') { //Other markers
		oth_markers = *((struct optitrack_other_markers_udp_packet *)(buffer+1));
		if( debug )
			printf("Other marker (iFrame %i, Latency %f ms, num_markers %i)\n", oth_markers.iFrame, oth_markers.Latency, oth_markers.nMarkers);

		int index = sizeof(struct optitrack_other_markers_udp_packet)+1;
		for (int j = 0; j<oth_markers.nMarkers; j++) {
			struct optitrack_marker_udp_packet marker;
			marker = *((struct optitrack_marker_udp_packet *)(buffer+index));
			index += sizeof(optitrack_marker_udp_packet);
			if (debug)
				printf(" - marker #%i: %f, %f, %f m\n", marker.ID, marker.x, marker.y, marker.z);
		}
	}
	else if (pck_type == 'T') { //Trackable
		rb = *((struct optitrack_rigid_body_udp_packet *)(buffer+1));


		Eigen::Matrix3d R_o;
		R_o << 0, 0, 1, 
				  1, 0, 0,
					0, 1, 0;


		//---Raw input
		Vector3d p_opt; // = Zeros;
		Vector4d q_opt; // = Zeros;
		p_opt << rb.x, rb.y, rb.z;
		q_opt << rb.qw, rb.qx, rb.qy, rb.qz;
		
		Matrix3d R_opt = utilities::QuatToMat( q_opt );

		if( output_ref_frame == "ned" ) {


			Matrix3d R_o_ned;
			R_o_ned <<       0,  0, 1,
											-1,  0, 0,
											 0, -1, 0;

			//Rotate optitrack data
			Vector3d p_opt2_ned = R_o_ned*p_opt;
			Matrix3d R_opt2_ned = R_o_ned*R_opt*R_o.transpose();
			Vector4d q_opt2_ned = utilities::MatToQuat(R_opt2_ned);



			pose.header.stamp = ros::Time::now();
			pose.pose.position.x = p_opt2_ned[0];
			pose.pose.position.y = p_opt2_ned[1];
			pose.pose.position.z = p_opt2_ned[2];


			pose.pose.orientation.w = q_opt2_ned[0];
			pose.pose.orientation.x = q_opt2_ned[1];
			pose.pose.orientation.y = q_opt2_ned[2];
			pose.pose.orientation.z = q_opt2_ned[3];
		}


		else if( output_ref_frame == "enu" ) {


			Matrix3d R_o_enu;
			R_o_enu << -1,  0, 0,
								  0,  0, 1,
									0,  1, 0;
			Vector3d p_opt2_enu = R_o_enu*p_opt;
			Matrix3d R_opt2_enu = R_o_enu*R_opt*R_o.transpose();
			Vector4d q_opt2_enu = utilities::MatToQuat(R_opt2_enu);

			//cout << "R_enu: " << R_opt2_enu << endl;
			pose.header.stamp = ros::Time::now();
			pose.pose.position.x = p_opt2_enu[0];
			pose.pose.position.y = p_opt2_enu[1];
			pose.pose.position.z = p_opt2_enu[2];


			pose.pose.orientation.w = q_opt2_enu[0];
			pose.pose.orientation.x = q_opt2_enu[1];
			pose.pose.orientation.y = q_opt2_enu[2];
			pose.pose.orientation.z = q_opt2_enu[3];
		}

		quad_pose.publish(pose);
		
		
		int index = sizeof(struct optitrack_rigid_body_udp_packet)+1;
		for (int j = 0; j<rb.nMarkers; j++) {
			struct optitrack_marker_udp_packet marker;
			marker = *((struct optitrack_marker_udp_packet *)(buffer+index));
			index += sizeof(optitrack_marker_udp_packet);
			printf(" - marker #%i: %f, %f, %f m\n", marker.ID, marker.x, marker.y, marker.z);
		}
	}
}



int listener::run() {
	/* Open Socket on port "port" */
	if ( !Open(port) ) {
		cout << "Socket: opening failed!" << endl;
		return -1;
	}
	while(ros::ok()) {
		Read();
		ros::spinOnce();
	}
}

/* main */
int main(int argc, char** argv) {
	ros::init(argc, argv, "optitrack_listener");
	listener listener_;
	listener_.run();
	return 0;
}
