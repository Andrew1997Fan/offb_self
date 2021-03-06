#include <iostream>
#include <ros/ros.h>
#include<thread>
#include<vector>
#include <Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include "autopilot.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "std_msgs/String.h"
#include <math.h>
#include "gps_transform.h"



#define error 0.1
double norm = 0;

autopilot::autopilot(gps_transform gps){
	this->gps = gps;
	double *pose_now = new double[3];
	double *target_now = new double[3];
	state = autopilot_state::not_flying;
	land_ok = false;
	

}
void autopilot::apriltag_update(double vector_x,double vector_y,double vector_z,double q_x,double q_y, double q_z, double q_w){
	this->vector_x = vector_x;
	this->vector_y = vector_y;
	this->vector_z = vector_z;
	this->q_x = q_x;
	this->q_y = q_y;
	this->q_z = q_z;
	this->q_w = q_w;
}

void autopilot::update(double *recent_pose){
	pose_now[0] = recent_pose[0];
	pose_now[1] = recent_pose[1];
	pose_now[2] = recent_pose[2];

	if (get_state() == autopilot_state::pose){
	}
	if (get_state() == autopilot_state::takeoff){
		takeoff();
		ROS_INFO_ONCE("start takeoff");
		if(is_arrived_z() == true){
			if(in_mission == true){
				state = autopilot_state::waypoint;
				ROS_INFO("go to waypoint %d",waypoint_num);
			}
			else{
				state = autopilot_state::pose;
				ROS_INFO("start pose control");
			}
		}
	}
	if(get_state() == autopilot_state::waypoint){
		double lat = waypoints[waypoint_num].latitude;
		double lon = waypoints[waypoint_num].longitude;
		double alt = waypoints[waypoint_num].altitude;

		this->gps.update(lat,lon,alt);
		this->gps.get_ENU(target_now);
		target_now[2] = 4;

		std::cout << "target_now[0]:" << target_now[0] << std::endl;
		std::cout << "target_now[1]:" << target_now[1] << std::endl;

		
		if(is_arrived_xy() == true){
			waypoint_num ++;
			if(waypoint_num != waypoints.size()){
				ROS_INFO("go to waypoint %d",waypoint_num);
			}
			else{
				ROS_INFO("all waypoints reached");
				state = autopilot_state::detection_and_move;
			}

		}
	}
	if(get_state() == autopilot_state::detection_and_move){
		detection_and_move(this->vector_x,this->vector_y,this->vector_z,this->q_x,this->q_y,this->q_z,this->q_w,this->q_imu_x,this->q_imu_y,this->q_imu_z,this->q_imu_w);
	}
	
	if (get_state() == autopilot_state::apriltag){
		ROS_INFO("On top of apriltag");
		state = autopilot_state::land;
	}
	
	if (get_state() == autopilot_state::land){
		land(this->vector_x,this->vector_y,this->vector_z,this->q_x,this->q_y,this->q_z,this->q_w,this->q_imu_x,this->q_imu_y,this->q_imu_z,this->q_imu_w);
		ROS_INFO_ONCE("start landing");
		if(pose_now[2]<=0.10){ //something strange here why it can pass this one
			land_ok = true;
			std::cout << "ready to disarm\n";
			state = autopilot_state::not_flying;
			if(get_state() == autopilot_state::not_flying){
				ROS_INFO_ONCE("disarm\n");
			}
			in_mission = false;
		}
	}
}

void autopilot::mission_stop(){
	if(in_mission == true){
		std::cout <<"mission stop!\n";
		state = autopilot_state::pose;
		in_mission = false;
		target_now[0] = pose_now[0];
		target_now[1] = pose_now[1];
		target_now[2] = pose_now[2];
	}
	else{
		std::cout << "not in mission\n";
	}	
}

void autopilot::mission_start(){
	if (in_mission == false){
		state = autopilot_state::takeoff;
		in_mission = true;
		waypoint_num = 0;
		pose_start[0] = pose_now[0];
		pose_start[1] = pose_now[1];
		pose_start[2] = pose_now[2];
	}
	else{
		std::cout << "already in mission\n";
	}
}
void autopilot::add_waypoint(double latitude,double longitude,double altitude){
    sensor_msgs::NavSatFix temp;
	temp.latitude = latitude;
	temp.longitude = longitude;
	temp.altitude = altitude;
	this->waypoints.push_back(temp);	
}

void autopilot::add_waypoint(sensor_msgs::NavSatFix msg){
	this->waypoints.push_back(msg);	
}

void autopilot::add_waypoint(double *msg){
    sensor_msgs::NavSatFix temp;
	temp.latitude = msg[0];
	temp.longitude = msg[1];
	temp.altitude = msg[2];
	this->waypoints.push_back(temp);	
}

void autopilot::show_waypoints(){
    sensor_msgs::NavSatFix temp;
	char str[100];
	std::cout << "show waypoints\n\n" ;
	for(int i=0;i<this->waypoints.size();i++){
        temp = this->waypoints[i];
		sprintf(str, "waypoint %d: \tlatitude:%lf\tlongitude:%lf\taltitude:%lf\n", i,temp.latitude,temp.longitude,temp.altitude);
		std::cout << str;
    }
}

void  autopilot::takeoff(){
	target_now[0] = pose_start[0];
	target_now[1] = pose_start[1];
	target_now[2] = 4;
}

// Apriltags
void autopilot::detection_and_move(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_imu_x,double q_imu_y,double q_imu_z,double q_imu_w){

		std::cout<<"in detection_and_move state"<<std::endl;
		if(this->vector_x != 0 && this->vector_y != 0){ //see apriltag
				if (is_arrived_z() == true){
					// Quadrotors in body frame				
					Eigen::Vector4d body_ENU;//Quadrotors pose_now
					body_ENU << pose_now[0],pose_now[1],pose_now[2],1;
					
					// fake apriltag
					Eigen::Vector4d fake_apriltag_tf;
					//fake_apriltag_tf <<-(this->vector_y+1.6),(this->vector_x+2.3),this->vector_z,1;
					fake_apriltag_tf <<this->vector_x,this->vector_y,this->vector_z,1;

					//camera to IMU
					Eigen::Matrix4d camera2IMU_transformation_matrix;//ignore camera imu hardware 
					camera2IMU_transformation_matrix << 0, 1, 0, 0,
														 1, 0, 0,     0.1,
														 0, 0,-1, -0.12,
														 0, 0, 0,     1;

					Eigen::Vector4d TC_IMU;
					TC_IMU = camera2IMU_transformation_matrix*fake_apriltag_tf;
					std::cout << "TC_IMU:" << TC_IMU << std::endl;

					

					//IMU quaternion
					Eigen::Quaterniond imu_quaternion(this->q_imu_x,this->q_imu_y,this->q_imu_z,this->q_imu_w);
					Eigen::Matrix3d imu_rotation_matrix;
					imu_rotation_matrix = imu_quaternion.toRotationMatrix();
					Eigen::Matrix4d Timu2w_transformation_matrix; //imu to world
					Timu2w_transformation_matrix << imu_rotation_matrix(0,0),imu_rotation_matrix(0,1),imu_rotation_matrix(0,2),pose_now[1],
												  imu_rotation_matrix(1,0),imu_rotation_matrix(1,1),imu_rotation_matrix(1,2),-pose_now[0], 
												  imu_rotation_matrix(2,0),imu_rotation_matrix(2,1),imu_rotation_matrix(2,2),pose_now[2],
												                         0,                       0,                       0,          1;

					//std::cout << "Timu2w" << Timu2w_transformation_matrix << std::endl;

					//IMU2world
					Eigen::Vector4d Timu_w;
					Timu_w = Timu2w_transformation_matrix*camera2IMU_transformation_matrix*fake_apriltag_tf;
					std::cout <<"Timu_w :" << Timu_w << std::endl;


					Eigen::Matrix4d w2w_new_transformation_matrix;
					w2w_new_transformation_matrix << 0, -1, 0, 0,
													1, 0, 0, 0,
													 0, 0, 1, 0,
													 0, 0, 0, 1;

					//apriltag pose_now in world frame
					//quadrotors target_now 
					Eigen::Vector4d apriltag_in_world;//target_now
					apriltag_in_world = w2w_new_transformation_matrix*Timu2w_transformation_matrix * camera2IMU_transformation_matrix* fake_apriltag_tf;



					std::cout << "apriltag_tf :" << fake_apriltag_tf << std::endl;
					
					//Quadrotors's pose in world frame
					std::cout << "pose_now[0]:" << body_ENU(0) << std::endl;
					std::cout << "pose_now[1]:" << body_ENU(1) << std::endl;
					std::cout << "apriltag_ENU(0):" << apriltag_in_world(0) << std::endl;
					std::cout << "apriltag_ENU(1):" << apriltag_in_world(1) << std::endl;



					//Feedback apriltag world frame vector to quadrotors's target now 
					target_now[0] = apriltag_in_world(0);
					target_now[1] = apriltag_in_world(1);

					std::cout<<"target_now[0]:"<<target_now[0]<<std::endl;
					std::cout<<"target_now[1]:"<<target_now[1]<<std::endl;
					
					
					double error_x = target_now[0]-body_ENU(0);
					double error_y = target_now[1]-body_ENU(1);
					std::cout << "errorq_x:" << error_x << std::endl;
					std::cout << "errorq_y:" << error_y << std::endl;

					norm = sqrt(std::pow(error_x,2) + std::pow(error_y,2));
					std::cout << "norm:" << norm << std::endl;



					if (norm<=0.45){
						ROS_INFO_ONCE("above apriltag and go on next state");
						state = autopilot_state::apriltag;
					}
					
					
					else{
						ROS_INFO_ONCE("not above apriltag still remain detection state");
						double error_x = target_now[0]-body_ENU(0);
						double error_y = target_now[1]-body_ENU(1);
						norm = sqrt(std::pow(error_x,2) + std::pow(error_y,2));
						std::cout << "norm:" << norm << std::endl;
						target_now[0] = pose_now[0] + error_x;
						target_now[1] = pose_now[1] + error_y;
						std::cout << "target_now[0]:" << target_now[0] << std::endl;
						std::cout << "target_now[1]:" << target_now[1] << std::endl;
					}

				}
				else{
					std::cout << "not in the right altitude" << std::endl;
				}


		}
		else{
			std::cout << "camera got nothing" <<std::endl;
		}
		
}

// adding apriltag info into landing state
void autopilot::land(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_imu_x,double q_imu_y,double q_imu_z,double q_imu_w){
	std::cout << "in landing state" <<std::endl;

	double error_x = target_now[0]-pose_now[0];
	double error_y = target_now[1]-pose_now[1];
	std::cout << "errorq_x:" << error_x << std::endl;
	std::cout << "errorq_y:" << error_y << std::endl;

	norm = sqrt(std::pow(error_x,2) + std::pow(error_y,2));
	std::cout << "norm:" << norm << std::endl;

	if(norm<0.1){ //abs(this->vector_x + 1.7) <= 0.3 && abs(this->vector_y + 1.2) <= 0.3
		ROS_INFO("start to land");

		target_now[2]=pose_now[2]-0.15; //target_now[2]-0.001
		std::cout << "norm : " << norm << std::endl;
		std::cout << "height_now[2]:" << pose_now[2] << std::endl;
		std::cout << "target_height_now[2]:" << target_now[2] << std::endl;
		std::cout << "pose_now[0]:" << pose_now[0] << std::endl;
		std::cout << "pose_now[1]:" << pose_now[1] << std::endl;
		std::cout << "target_now[0]:" << target_now[0] << std::endl;
		std::cout << "target_now[1]:" << target_now[1] << std::endl;
		std::cout << "tf_x:" << vector_x << std::endl;
		std::cout << "tf_y:" << vector_y << std::endl;


	}
	else if (norm>0.45)
	{
		ROS_INFO("too far away from apriltag");
		state = autopilot_state::detection_and_move;
	}
	
	else{
		ROS_INFO("compensate xy");

		double error_x = target_now[0]-pose_now[0];
		double error_y = target_now[1]-pose_now[1];
		norm = sqrt(std::pow(error_x,2) + std::pow(error_y,2));

		target_now[0] = pose_now[0] + error_x;
		target_now[1] = pose_now[1] + error_y;
		std::cout << "pose_now[0]:" << pose_now[0] << std::endl;
		std::cout << "pose_now[1]:" << pose_now[1] << std::endl;
		std::cout << "target_now[0]:" << target_now[0] << std::endl;
		std::cout << "target_now[1]:" << target_now[1] << std::endl;
		std::cout << "norm : " << norm << std::endl;
	}
}


bool autopilot::is_arrived_xy(){
	if(abs(target_now[0]-pose_now[0])<error && abs(target_now[1]-pose_now[1])<error){
		return true;
	}else{
		return false;
	}
}
bool autopilot::is_arrived_z(){
	if(target_now[2]-pose_now[2]<error){
		return true;
	}else{
		return false;
	}
}
bool autopilot::get_land_ok(){
	return land_ok;
}

autopilot_state autopilot::get_state(){
	return state;
}

double* autopilot::get_target_now(){
	return target_now;
}
