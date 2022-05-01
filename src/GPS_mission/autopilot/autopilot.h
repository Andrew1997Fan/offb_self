#ifndef AUTOPILOT
#define AUTOPILOT
#include <math.h>
#include "gps_transform.h"
#include "tf2_msgs/TFMessage.h"
#include "std_msgs/String.h"


enum autopilot_state{not_flying,takeoff,pose,waypoint,land,apriltag,detection_and_move};
//void tf_Callback(const tf2_msgs::TFMessage::ConstPtr &msg);
class autopilot{
	public:
		autopilot(gps_transform gps);
		
		void update(double *recent_pose);
		void apriltag_update(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w);
		//add waypoint by different datatype
		void add_waypoint(sensor_msgs::NavSatFix);
		void add_waypoint(double*);
		void add_waypoint(double latitude,double longitude ,double altitude);
		
		void show_waypoints();
		
		void takeoff();
		void land(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_odometry_x,double q_odometry_y,double q_odometry_z,double q_odometry_w,double odometry_x,double odometry_y, double odometry_z);
		void mission_start();
		void mission_stop();
		//Apriltags
		void apriltag(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_odometry_x,double q_odometry_y,double q_odometry_z,double q_odometry_w,double odometry_x,double odometry_y, double odometry_z);
		void detection_and_move(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_odometry_x,double q_odometry_y,double q_odometry_z,double q_odometry_w,double odometry_x,double odometry_y, double odometry_z);


		bool is_arrived_xy();
		bool is_arrived_z();
		bool get_land_ok();
		
		autopilot_state get_state();
		
		double* get_target_now();
	private:
		std::vector<sensor_msgs::NavSatFix> waypoints;
		autopilot_state state;
		int waypoint_num = 0;	

		double vector_x,vector_y,vector_z;
//		double camera_err_x,camera_err_y;

		//Quaternion rotation
		//double R11,R12,R13,R21,R22,R23,R31,R32,R33;
		double q_x,q_y,q_z,q_w;
		double q_odometry_x,q_odometry_y,q_odometry_z,q_odometry_w,odometry_x,odometry_y,odometry_z;

		double target_now[3];
		double pose_now[3];
		double pose_start[3];
		
		double error_x,error_y;
		
		bool land_ok;
		bool in_mission = false;
		gps_transform gps;
};
#endif
