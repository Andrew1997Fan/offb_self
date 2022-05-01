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
		target_now[2] = 3;

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
		detection_and_move(this->vector_x,this->vector_y,this->vector_z,this->q_x,this->q_y,this->q_z,this->q_w,this->q_odometry_x,this->q_odometry_y,this->q_odometry_z,this->q_odometry_w,this->odometry_x,this->odometry_y,this->odometry_z);
	}
	
	if (get_state() == autopilot_state::apriltag){
		ROS_INFO("On top of apriltag");
		state = autopilot_state::land;
	}
	
	if (get_state() == autopilot_state::land){
		land(this->vector_x,this->vector_y,this->vector_z,this->q_x,this->q_y,this->q_z,this->q_w,this->q_odometry_x,this->q_odometry_y,this->q_odometry_z,this->q_odometry_w,this->odometry_x,this->odometry_y,this->odometry_z);
		ROS_INFO_ONCE("start landing");
		if(pose_now[2]<=0.15){ //something strange here why it can pass this one
			land_ok = true;
			std::cout << "ready to disarm\n";
			state = autopilot_state::not_flying;
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
	target_now[2] = 3;
}

// Apriltags
void autopilot::detection_and_move(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_odometry_x,double q_odometry_y,double q_odometry_z,double q_odometry_w,double odometry_x,double odometry_y, double odometry_z){

		std::cout<<"in detection_and_move state"<<std::endl;
		if(this->vector_x != 0 && this->vector_y != 0){ //see apriltag
				if (is_arrived_z() == true){

					//if height is ok
					//update apriltag enu coordinate 
					//calculate Rotation matrix from quaternion
					/*
					this->R11 = 1-2*(std::pow(q_y,2)+std::pow(q_z,2));
					this->R12 = 2*(q_x*q_y-q_w*q_z);
					this->R13 = 2*(q_w*q_y+q_x*q_z);
					this->R21 = 2*(q_x*q_y+q_w*q_z);
					this->R22 = 1-2*(std::pow(q_x,2)+std::pow(q_z,2));
					this->R23 = 2*(q_y*q_z-q_w*q_x);
					this->R31 = 2*(q_x*q_z-q_w*q_y);
					this->R32 = 2*(q_y*q_z+q_w*q_x);
					this->R33 = 1-2*(std::pow(q_x,2)+std::pow(q_y,2));
					target_now[0] = this->R11*pose_now[0] + this->R12*pose_now[1] + this->R13*pose_now[2] - (this->vector_y+1.2) ;
					target_now[1] = this->R21*pose_now[0] + this->R22*pose_now[1] + this->R23*pose_now[2] - (this->vector_x+1.7);
					target_now[2] = R31*pose_now[0] + R32*pose_now[1] + R33*pose_now[2];
					*/
					

					// Quadrotors in body frame				
					Eigen::Vector4d body_ENU;//Quadrotors pose_now
					body_ENU << pose_now[0],pose_now[1],pose_now[2],1;
					
					// P_a in apriltag frame
					Eigen::Vector3d apriltag_translation;
					apriltag_translation << this->vector_x,this->vector_y,this->vector_z;
					Eigen::Vector4d apriltag_tf;
					apriltag_tf << this->vector_x,this->vector_y,this->vector_z,1;


					//Quaternion to Rotation matrix (camera in apriltag frame so need a transformation matrix)
					Eigen::Quaterniond apriltag2camera_quaternion(this->q_w,this->q_x,this->q_y,this->q_z);
					Eigen::Matrix3d apriltag2camera_rotation_matrix;
					apriltag2camera_rotation_matrix = apriltag2camera_quaternion.toRotationMatrix();
					//Tc_a
					Eigen::Matrix4d apriltag2camera_transformation_matrix;
					apriltag2camera_transformation_matrix << apriltag2camera_rotation_matrix(0,0),apriltag2camera_rotation_matrix(0,1),apriltag2camera_rotation_matrix(0,2),vector_x,
					 						 				 apriltag2camera_rotation_matrix(1,0),apriltag2camera_rotation_matrix(1,1),apriltag2camera_rotation_matrix(1,2),vector_y,
					 						 				 apriltag2camera_rotation_matrix(2,0),apriltag2camera_rotation_matrix(2,1),apriltag2camera_rotation_matrix(2,2),vector_z,
					 						                    				0,                   0,                   0,       1;

					// //Ta_c need to imply the transformation inverse policy
					// Eigen::Matrix4d camera2apriltag_transformation_matrix;
					// Eigen::Matrix3d a2c_rotation_matrix_transpose;
					// Eigen::Vector3d a2c_translation_vector_transpose;
					// a2c_rotation_matrix_transpose = apriltag2camera_rotation_matrix.transpose();
					// a2c_translation_vector_transpose = (-1)*a2c_rotation_matrix_transpose * apriltag_translation;
					// camera2apriltag_transformation_matrix = apriltag2camera_transformation_matrix;
					

					//assume camera and quadrotors aligned
					//Tc_q
					Eigen::Matrix4d camera2quadrotors_transformation_matrix;
					camera2quadrotors_transformation_matrix <<  0, -1, 0,     0,
																-1, 0, 0, -0.12,
															    0,  0,-1, -0.12,
																0,  0, 0,     1;



					//IMU quaternion (odometry = imu)
					Eigen::Quaterniond odometry_quaternion(this->q_odometry_x,this->q_odometry_y,this->q_odometry_z,this->q_odometry_w);
					Eigen::Matrix3d odometry_rotation_matrix;
					odometry_rotation_matrix = odometry_quaternion.toRotationMatrix();
					Eigen::Matrix4d Tq2w_transformation_matrix; //quadrotors to world frame (imu to world)
					Tq2w_transformation_matrix << odometry_rotation_matrix(0,0),odometry_rotation_matrix(0,1),odometry_rotation_matrix(0,2),pose_now[1],
												  odometry_rotation_matrix(1,0),odometry_rotation_matrix(1,1),odometry_rotation_matrix(1,2),pose_now[0], 
												  odometry_rotation_matrix(2,0),odometry_rotation_matrix(2,1),odometry_rotation_matrix(2,2),pose_now[2],
												                         0,                       0,                       0,          1;

					
					Eigen::Matrix4d quadrotors2imu_rotation_matrix;
					quadrotors2imu_rotation_matrix << -1, 0, 0, 0,
													   0,-1, 0, 0,
													   0, 0,-1, 0,
													   0, 0, 0, 1;


					//Rotation quadrotors to world frame R_q2w from numerical solution 
					// Eigen::Matrix3d R_q2w_matrix;
					// R_q2w_matrix << 7.9413,-1.6694,0,
					// 				4.2528, 0.7519,0,
					// 					0,       0,0;
					// Eigen::Matrix4d R_q2w_transformation_matrix; // translation vector 
					// R_q2w_transformation_matrix << R_q2w_matrix(0,0),R_q2w_matrix(0,1),R_q2w_matrix(0,2),target_now[0],
					// 							   R_q2w_matrix(1,0),R_q2w_matrix(1,1),R_q2w_matrix(1,2),target_now[1],
					// 							   R_q2w_matrix(2,0),R_q2w_matrix(2,1),R_q2w_matrix(2,2),target_now[2],
					// 							   				   0, 			     0,                0,          1;




					//apriltag pose_now in world frame
					//quadrotors target_now 
					Eigen::Vector4d apriltag_in_world;//target_now
					apriltag_in_world = Tq2w_transformation_matrix * quadrotors2imu_rotation_matrix * camera2quadrotors_transformation_matrix * apriltag_tf;
					// Pa_W = TI_W*TB_I*TC_B*Pa_C


					std::cout << "apriltag_tf :" << apriltag_tf << std::endl;
					
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

					//Update Quadrotors pose_now to target_now
					//let target_now don't update 
					
					
					// target_now[0] = pose_now[0] + 0.5*errorq_y;//- errorq_y;
					// target_now[1] = pose_now[1] + 0.5*errorq_x;apriltag and go on ne//- errorq_x;
				
				
					//target_now[0] = pose_now[0] + 0.5*errorq_y;//- errorq_y;
					//target_now[1] = pose_now[1] + 0.5*errorq_x;//- errorq_x;
					//std::cout << "target_now[0]:" << target_now[0] << std::endl;
					//std::cout << "target_now[1]:" << target_now[1] << std::endl;
					//if

					if (norm<=2.0){
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

					/*target_now[0] = pose_now[0] - (this->vector_y+1.2);
					target_now[1] = pose_now[1] - (this->vector_x+1.7);
					std::cout << "pose_now[0]:" << pose_now[0] << std::endl;
					std::cout << "pose_now[1]:" << pose_now[1] << std::endl;
					std::cout << "target_now[0]:" << target_now[0] << std::endl;
					std::cout << "target_now[1]:" << target_now[1] << std::endl;
					std::cout << "cam_err_x:" << this->vector_x << std::endl;
					std::cout << "cam_err_y:" << this->vector_y << std::endl;

					//gps.ENU_2_WGS84_update(target_now)
					*/

					//target_now[0] = pose_now[0] - (this->vector_y+1.2);
					//target_now[1] = pose_now[1] - (this->vector_x+1.7);
					/*std::cout << "pose_now[0]:" << pose_now[0] << std::endl;
					std::cout << "pose_now[1]:" << pose_now[1] << std::endl;
					std::cout << "target_now[0]:" << target_now[0] << std::endl;
					std::cout << "target_now[1]:" << target_now[1] << std::endl;
					*/
					// std::cout << "cam_err_x:" << this->vector_x << std::endl;
					// std::cout << "cam_err_y:" << this->vector_y << std::endl;
				
					// // camera for px4_camera calibration
					// double x_dis = this->vector_x + 1.7 ;
					// double y_dis = this->vector_y + 1.2 ;
					// norm = sqrt((x_dis)*(x_dis)+(y_dis)*(y_dis));
					// std::cout << "norm :" << norm << std::endl;

					// if(abs(target_now[0] = pose_now[0])<=0.2 && abs(target_now[1] = pose_now[1])<=0.2){   //make sure apriltag at center  of camera
					// 	ROS_INFO("above the middle of apriltag");
					// 	//target_now[0] = pose_now[0];
					// 	//target_now[1] = pose_now[1];
					// 	std::cout << "height:" << apriltag_ENU(2) << std::endl;
					// 	std::cout << "error_x:" << abs(apriltag_ENU(0)-body_ENU(0)) << std::endl;
					// 	std::cout << "error_y:" << abs(apriltag_ENU(1)-body_ENU(1)) << std::endl;

					// 	state = autopilot_state::apriltag;
					// }
				}
				/*else{
					ROS_INFO("q_x>0.99");
					apriltag_ENU(0) = body_ENU(0) - (this->vector_y+1.2);
					apriltag_ENU(1) = body_ENU(1) - (this->vector_x+1.7);
					std::cout << "pose_now[0]:" << body_ENU(0) << std::endl;
					std::cout << "pose_now[1]:" << body_ENU(1) << std::endl;
					std::cout << "target_now[0]:" << target_now[0] << std::endl;
					std::cout << "target_now[1]:" << target_now[1] << std::endl;
					std::cout << "cam_err_x:" << this->vector_x << std::endl;
					std::cout << "cam_err_y:" << this->vector_y << std::endl;

					if(abs(target_now[0]-pose_now[0])<=0.1 && abs(target_now[1]-pose_now[1]<=0.1)){   //make sure apriltag at center  of camera
						ROS_INFO("above the middle of apriltag");
						//target_now[0] = pose_now[0];
						//target_now[1] = pose_now[1];
						std::cout << "height:" << target_now[2] << std::endl;
						std::cout << "error_x:" << abs(target_now[0]-pose_now[0]) << std::endl;
						std::cout << "error_y:" << abs(target_now[1]-pose_now[1]) << std::endl;

						state = autopilot_state::apriltag;
						}

				}*/
				/*else{
					ROS_INFO("quadcopter not in the right height");
					std::cout << "pose_now[2]:" << pose_now[2] << std::endl;
					std::cout << "target_now[2] :" << target_now[2]<< std::endl;
					target_now[0] = pose_now[0];
					target_now[1] = pose_now[1];
					target_now[2] = 3;
				}*/
		}
		else{
			std::cout << "camera got nothing" <<std::endl;
		}
		
}

// adding apriltag info into landing state
void autopilot::land(double vector_x,double vector_y,double vector_z,double q_x,double q_y,double q_z,double q_w,double q_odometry_x,double q_odometry_y,double q_odometry_z,double q_odometry_w,double odometry_x,double odometry_y, double odometry_z){
	std::cout << "in landing tate" <<std::endl;
	// double x_dis = this->vector_x + 1.7 ;
	// double y_dis = this->vector_y + 1.2 ;
	// norm = sqrt((x_dis)*(x_dis)+(y_dis)*(y_dis));

	if(norm<1.5){ //abs(this->vector_x + 1.7) <= 0.3 && abs(this->vector_y + 1.2) <= 0.3
		ROS_INFO("start to land");		
		target_now[2] = 0;
		std::cout << "norm : " << norm << std::endl;
		std::cout << "height_now[2]:" << pose_now[2] << std::endl;
		std::cout << "target_height_now[2]:" << target_now[2] << std::endl;
		/*if(norm<0.25){
			ROS_INFO("start to land");		
			target_now[2] = pose_now[2] - 0.3;
			std::cout << "norm : " << norm << std::endl;
			std::cout << "height_now[2]:" << pose_now[2] << std::endl;
			std::cout << "target_height_now[2]:" << target_now[2] << std::endl;
			
		}*/
	}
	else if (norm>3.0)
	{
		ROS_INFO("too far away from apriltag");
		state = autopilot_state::detection_and_move;
	}
	
	else{
		ROS_INFO("compensate xy");
		//double x_dis = this->vector_x + 1.7 ;
		//double y_dis = this->vector_y + 1.2 ;
		//norm = sqrt((x_dis)*(x_dis)+(y_dis)*(y_dis));
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
