#include <iostream>
#include <ros/ros.h>
#include<thread>
#include<vector>
#include<sensor_msgs/NavSatFix.h>
#include "autopilot.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "std_msgs/String.h"
#include <math.h>

#define error 0.1

double norm;

autopilot::autopilot(gps_transform gps){
	this->gps = gps;
	double *pose_now = new double[3];
	double *target_now = new double[3];
	state = autopilot_state::not_flying;
	land_ok = false;
	

}
void autopilot::apriltag_update(double vector_x,double vector_y){
	this->vector_x = vector_x;
	this->vector_y = vector_y;
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
	else if(get_state() == autopilot_state::detection_and_move){
		detection_and_move(this->vector_x,this->vector_y);
	}
	
	if (get_state() == autopilot_state::apriltag){
		ROS_INFO("Arrived the place of apriltag");
		state = autopilot_state::land;
	}
	
	if (get_state() == autopilot_state::land){
		land(this->vector_x,this->vector_y);
		ROS_INFO_ONCE("start landing");
		if(is_arrived_z() == true){
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
void autopilot::detection_and_move(double vector_x,double vector_y){

		std::cout<<"in detection_and_move state"<<std::endl;
		if(this->vector_x != 0 && this->vector_y != 0){

				if (abs(target_now[2]-3)<0.5){

					target_now[0] = pose_now[0] - (this->vector_y+1.4);
					target_now[1] = pose_now[1] - (this->vector_x+1.8);
					std::cout << "pose_now[0]:" << pose_now[0] << std::endl;
					std::cout << "pose_now[1]:" << pose_now[1] << std::endl;
					std::cout << "target_now[0]:" << target_now[0] << std::endl;
					std::cout << "target_now[1]:" << target_now[1] << std::endl;
					std::cout << "cam_err_x:" << this->vector_x << std::endl;
					std::cout << "cam_err_y:" << this->vector_y << std::endl;
				
					// camera for px4_camera calibration
					double x_dis = this->vector_x + 1.8 ;
					double y_dis = this->vector_y + 1.4 ;
					norm = sqrt((x_dis)*(x_dis)+(y_dis)*(y_dis));
					std::cout << "norm :" << norm << std::endl;

					if(norm<0.4){
						ROS_INFO("above apriltag");
						target_now[0] = pose_now[0];
						target_now[1] = pose_now[1];
						std::cout << "height:" << target_now[2] << std::endl;
						state = autopilot_state::apriltag;
					}
				}else{
					ROS_INFO("camera got nothing");
					target_now[0] = pose_now[0];
					target_now[1] = pose_now[1];
					target_now[2] = 3;
				}
				
				


		}
		else{
			std::cout << "camera got nothing" <<std::endl;
			//state = autopilot_state::apriltag;
		}
		
}

// adding apriltag info into landing state
void autopilot::land(double vector_x,double vector_y){

	if(this->vector_x <= -1.3 && this->vector_y <= -0.9){
		
		if(norm<0.18){
			ROS_INFO("start to land");		
			target_now[2] = pose_now[2] - 0.2;
			std::cout << "norm : " << norm << std::endl;
			std::cout << "height_now[2]:" << pose_now[2] << std::endl;
			std::cout << "target_height_now[2]:" << target_now[2] << std::endl;
			
		}
		else{
			ROS_INFO("land with compensating xy");
			double x_dis = this->vector_x + 1.8 ;
			double y_dis = this->vector_y + 1.4 ;
			norm = sqrt((x_dis)*(x_dis)+(y_dis)*(y_dis));

			target_now[0] = pose_now[0] - (this->vector_y+1.4);
			target_now[1] = pose_now[1] - (this->vector_x+1.8);
			target_now[2] = pose_now[2] - 0.2;
			

			std::cout << "height_now[2]:" << pose_now[2] << std::endl;
			std::cout << "target_height_now[2]:" << target_now[2] << std::endl;
			std::cout << "norm : " << norm << std::endl;

		}

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
