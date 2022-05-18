#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandTOL.h>

#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
//#include <Eigen/Dense>

#include "gps_transform.h"
#include "autopilot.h"

#define XY_VEL_MAX 0.5
//gain
#define KPx 0.3f//1   3
#define KPy 0.3f//1   3
#define KPz 0.3f//
#define KProll 0.3f//1  2

bool init=0;
double vector_x = 0;
double vector_y = 0;
double vector_z = 0;
double q_x = 0;
double q_y = 0;
double q_z = 0;
double q_w = 0;
double q_odometry_x = 0;
double q_odometry_y = 0;
double q_odometry_z = 0;
double q_odometry_w = 0;
double odometry_x = 0;
double odometry_y = 0;
double odometry_z = 0;
//
gps_transform gps;
//autopilot ap(gps);


using namespace std;
struct vir	//virtual leader pose
{
    float roll;
    float x;
    float y;
    float z;
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped offset;
geometry_msgs::PoseStamped host_mocap;


void gps_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	double latitude = msg->latitude;
	double longitude = msg->longitude;
	double altitude = msg->altitude;
	//set home point
	if(gps.is_init() == false){ 
		
		gps.set_home_longitude_latitude(latitude,longitude,altitude);
	}
	
    	ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
    
	gps.update(latitude,longitude,altitude);
	/*
	double ecef_msg[3];
	gps.get_ECEF(ecef_msg);
	double enu_msg[3];
	gps.get_ENU(enu_msg);
	*/	
}

void tf_Callback(const tf2_msgs::TFMessage::ConstPtr &msg){
	vector_x = msg->transforms.back().transform.translation.x;
	vector_y = msg->transforms.back().transform.translation.y;
	vector_z = msg->transforms.back().transform.translation.z;
	q_x = msg->transforms.back().transform.rotation.x;
	q_y = msg->transforms.back().transform.rotation.y;
	q_z = msg->transforms.back().transform.rotation.z;
	q_w = msg->transforms.back().transform.rotation.w;

	//ap.apriltag_update(vector_x,vector_y);
	//ap.detection_and_move(vector_x,vector_y);

}
void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg){
	q_odometry_x = msg->pose.pose.orientation.x;
	q_odometry_y = msg->pose.pose.orientation.y;
	q_odometry_z = msg->pose.pose.orientation.z;
	q_odometry_w = msg->pose.pose.orientation.w;
	odometry_x = msg->pose.pose.position.x;
	odometry_y = msg->pose.pose.position.y;
	odometry_z = msg->pose.pose.position.z;

	std::cout << "q_odometry_x :" << q_odometry_x << std::endl;
	std::cout << "q_odometry_y :" << q_odometry_y << std::endl;
	std::cout << "q_odometry_z :" << q_odometry_z << std::endl;
	std::cout << "q_odometry_w :" << q_odometry_w << std::endl;
}
void follow(double* desired , double* recent, geometry_msgs::TwistStamped* vs, float dis_x, float dis_y)
{
	float err_x, err_y, err_z, err_roll;
	float u_x, u_y, u_z, u_roll;
/*


		
	err_roll = desired[3] - tf::getYaw(temp.pose.orientation);
	if(err_roll>pi)
		err_roll = err_roll - 2*pi;
	else if(err_roll<-pi)
		err_roll = err_roll + 2*pi;

*/
	err_x = desired[0] - recent[0];
	err_y = desired[1] - recent[1];
	err_z = desired[2] - recent[2];

	u_x = KPx*err_x;
	u_y = KPy*err_y;
	u_z = KPz*err_z;
	//u_roll = KProll*err_roll;

//	set upper bound	
	u_x = u_x > XY_VEL_MAX ? XY_VEL_MAX : u_x;
	u_y = u_y > XY_VEL_MAX ? XY_VEL_MAX : u_y;
	u_z = u_z > XY_VEL_MAX ? XY_VEL_MAX : u_z;
	u_x = u_x < -XY_VEL_MAX ? -XY_VEL_MAX : u_x;
	u_y = u_y < -XY_VEL_MAX ? -XY_VEL_MAX : u_y;
	u_z = u_z < -XY_VEL_MAX ? -XY_VEL_MAX : u_z;

	vs->twist.linear.x = u_x;
	vs->twist.linear.y = u_y;
	vs->twist.linear.z = u_z;
	vs->twist.angular.z = u_roll;

}
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_follow_test");
    ros::NodeHandle nh;
//Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("/mavros/state", 10, state_cb);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, gps_pos_cb);	//gps position
	ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/Odometry/in", 10,odometry_cb );	//odometry orientation
	ros::Subscriber sub = nh.subscribe("tf", 1000, tf_Callback);//callback tf topic

//Publisher
	ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
//Service
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(100);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
//	mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }

    
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vs;
	vir vir1;

    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;


    //send a few setpoints before starting

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while(gps.is_init()==false){
		ros::spinOnce();
        rate.sleep();
    }
    autopilot ap(gps);
	//ap.add_waypoint(47.3977419,8.5455936,535.150915657); //gazebo(0,0,0)
	//ap.add_waypoint(47.3977438,8.5456034,535.120903528);//gazebo(0.7,0.2,0)
	//ap.add_waypoint(47.397748,8.5456032,535.207905821);
	ap.add_waypoint(24.7869119,120.9934255,120.577500361);
	//ap.add_waypoint(47.3977871,8.5455936,535.3093731);//gazebo(0,5,0)
	//ap.add_waypoint(47.3977422,8.5456602,535.327822807);//gazebo(5,0,0,0)
	//ap.add_waypoint(24.7866673,120.9931711,106.093341741);
	ap.show_waypoints();
	char check;
	cout <<"start to fly?[y\\n]";
	while(cin >> check){
		if(check == 'n')
			return 0;
		else if(check == 'y')
			break;
		cout <<"start to fly?[y\\n]";
	}
    local_vel_pub.publish(vs);
	if (current_state.mode != "OFFBOARD"){
		if( set_mode_client.call(offb_set_mode) &&
			offb_set_mode.response.mode_sent) {
		ROS_INFO("Offboard enabled");
		}
	} 


	if (!current_state.armed){
		if( arming_client.call(arm_cmd) &&
				arm_cmd.response.success) {
			ROS_INFO("Vehicle armed");
		}
	}

    while (ros::ok()) {

		double now_pos[3];
		gps.get_ENU(now_pos);
		ap.update(now_pos);
		ap.apriltag_update(vector_x,vector_y,vector_z,q_x,q_y,q_z,q_w);
		

		if(ap.get_state() == autopilot_state::not_flying){
			vs.twist.linear.x = 0;
			vs.twist.linear.y = 0;
			vs.twist.linear.z = 0;
			vs.twist.angular.z = 0;
		}
		else{
			follow(ap.get_target_now(),now_pos,&vs,0,0);
		}
		if(ap.get_land_ok() == true){
	   		mavros_msgs::CommandTOL land_cmd;
	   		land_cmd.request.yaw = 0;
	    	land_cmd.request.latitude = 0;
	    	land_cmd.request.longitude = 0;
	    	land_cmd.request.altitude = 0;
	    	land_client.call(land_cmd);
			rate.sleep();
			break;	
		}

        int c = getch();
	//ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                vir1.z += 0.05;
                break;
            case 66:    // key down
                vir1.z += -0.05;
                break;
            case 67:    // key CW(->)
                vir1.roll -= 0.03;
                break;
            case 68:    // key CCW(<-)
                vir1.roll += 0.03;
                break;
			case 116:    // keyboard(t)		start
                ap.mission_start();
                break;
			case 119:    // key foward
                vir1.x += 0.05;
                break;
            case 120:    // key back
                vir1.x -= 0.05;
                break;
            case 97:    // key left
                vir1.y += 0.05;
                break;
            case 100:    // key right
                vir1.y -= 0.05;
                break;
            case 112:    // keyboard(p)		stop at recent position
            {
				std::cout <<"im here"<<std::endl;
				ap.mission_stop();
				break;
			}
            case 113:    // key board(q)	kill by keyboard
			{
				offb_set_mode.request.custom_mode = "MANUAL";
				set_mode_client.call(offb_set_mode);
				arm_cmd.request.value = false;
				arming_client.call(arm_cmd);
                break;
			}
	    	case 115:    // key right
			{
				vir1.x = offset.pose.position.x;
				vir1.y = offset.pose.position.y;
				vir1.z = offset.pose.position.z+0.2;
				vir1.roll = 0;
						break;
			}
			case 108:    // key_board(l)	land
			{
			    mavros_msgs::CommandTOL land_cmd;
			    land_cmd.request.yaw = 0;
			    land_cmd.request.latitude = 0;
			    land_cmd.request.longitude = 0;
			    land_cmd.request.altitude = 0;
			    land_client.call(land_cmd);
            	break;
			}
            case 63:
                return 0;
                break;
            }
        }
	
	


	local_vel_pub.publish(vs);

        ros::spinOnce();
	
        rate.sleep();
    }

    return 0;
}

