#include "ros/ros.h"

#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"

#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
//#include "TooN/TooN.h"
//#include "robohelper/robohelper.hpp"

#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "utils.h"

#ifndef O_LISTENER_H_
#define O_LISTENER_H_

#define OPTITRACK_BUF_SIZE 1000
#define OPTITRACK_PORT_DEF 9030

using namespace std;

struct optitrack_rigid_body_udp_packet {
	int ID;
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
	int iFrame;
	float Latency;
	int nMarkers;
};
struct optitrack_other_markers_udp_packet {
	int iFrame;
	float Latency;
	int nMarkers;
};
struct optitrack_marker_udp_packet {
	int ID;
	float x;
	float y;
	float z;
};

/*typedef struct OPTITRACK_DATA {
	int ID;
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
	int iFrame;
	float Latency;
} OPTITRACK_DATA;
*/

class listener {
public:
	listener();
	~listener();
	bool Open(int port_number);
	bool Read();
	int run();
private:
	int opt_socket;
	
	tf::tfMessage tfPublish;
	geometry_msgs::TransformStamped stampedPublish;
	std::stringstream intToString;
	int rlen, slen;
	unsigned char buffer[1024];
	ros::Publisher tfTopic;
	ros::Publisher quad_pose;
	sockaddr_in si_me, si_other;
	optitrack_rigid_body_udp_packet optitrack_data;
	ros::NodeHandle nodeHandle;	
	int port;
	bool blocking;
	bool debug;
	struct optitrack_rigid_body_udp_packet rb;
	struct optitrack_other_markers_udp_packet oth_markers;

	string output_topic_name;
	string output_ref_frame;

};

#endif /* O_LISTENER_H_ */
