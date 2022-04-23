#include <iostream>
#include <gps_transform.h>

using namespace std;
float gps_transform::N(float phi){  //degree to rad
	float e;
	e = 1 - b*b/(a*a); 
	return a/sqrt(1 - (sin(phi/180*pi)*e)*(sin(phi/180*pi)*e));
}

void gps_transform::update(double latitude, double longitude, double altitude){
	this->latitude = latitude;	
	this->longitude = longitude;	
	this->altitude = altitude;
	double ECEF_pose[3];	
	this->ECEF_update(latitude ,longitude,altitude, ECEF_pose);
	double ENU_pose[3];	
	this->ECEF_2_ENU_update(ECEF_pose[0],ECEF_pose[1],ECEF_pose[2],ENU_pose);
	//double result[3]; //aprilta enu_2_wgs84 
}

bool gps_transform::is_init(){
	return init_flag;
}

void gps_transform::set_home_longitude_latitude(double latitude ,double longitude,double altitude)
{

	this->home_longitude = longitude;
	this->home_latitude = latitude;
	this->home_height_msl = altitude;
	
	double ECEF_pose[3];	
	ECEF_update(latitude , longitude, altitude,ECEF_pose);
	
	this->home_ecef_x = this->X;
	this->home_ecef_y = this->Y;
	this->home_ecef_z = this->Z;
	
	this->r11 = -this->sin_lambda;
	this->r12 = this->cos_lambda;
	this->r13 = 0;
	this->r21 = -this->cos_lambda * this->sin_phi;
	this->r22 = -this->sin_lambda * this->sin_phi;
	this->r23 = this->cos_phi;
	this->r31 = this->cos_phi*this->cos_lambda;
	this->r32 = this->cos_phi*this->sin_lambda;
	this->r33 = this->sin_phi;
	this->init_flag = true;
	
}
void gps_transform::ECEF_update(double latitude ,double longitude,double altitude,double* ECEF_pose){
	this->sin_lambda = sin((longitude/180)*pi);
	this->cos_lambda = cos((longitude/180)*pi);
	this->sin_phi = sin((latitude/180)*pi);
	this->cos_phi = cos((latitude/180)*pi);
    this->X = (this->N(latitude)+altitude)*this->cos_phi*this->cos_lambda;
    this->Y = (this->N(latitude)+altitude)*this->cos_phi*this->sin_lambda;
    this->Z = (b*b/(a*a)*this->N(latitude)+altitude)*this->sin_phi;
	ECEF_pose[0] = this->X;
	ECEF_pose[1] = this->Y;
	ECEF_pose[2] = this->Z;

}

void gps_transform::ECEF_2_ENU_update(double X,double Y,double Z,double *ENU_pose){
	double dx =this-> X - home_ecef_x;
	double dy =this-> Y - home_ecef_y;
	double dz =this-> Z - home_ecef_z;
	//cout << "delta(new) \tx:"<<dx<<"\ty:"<<dy<<"\tz:"<<dz<<endl;
	//ENU
	this->x_enu = (this->r11 * dx) + (this->r12 * dy) + (this->r13 * dz);
	this->y_enu = (this->r21 * dx) + (this->r22 * dy) + (this->r23 * dz);
	//this->z_enu = (this->r31 * dx) + (this->r32 * dy) + (this->r33 * dz) ; //barometer of height sensor
	this->z_enu = this->altitude - this->home_height_msl; 
	ENU_pose[0]=x_enu;
	ENU_pose[1]=y_enu;
	ENU_pose[2]=z_enu;
}                    	

//ENU_2_WGS84
/*void gps_transform::ENU_2_WGS84_update(double x_east, double y_north, double z_up, double lati, double longti, double height, double *result){
	//double Pi = std::acos(-1.0);
    //double a = 6378137.0;
    //double b = 6356752.3142;
	
	//double result[3]; //aprilta enu_2_wgs84

    double f = (a-b)/a;
    double e_sq = f*(2-f);

    double lamb = lati *pi / 180.0;
    double phi = longti * pi /180.0;
    double sl = std::sin(lamb);
    double N = a/std::sqrt(1 - e_sq *sl *sl);
    double sin_lambda = std::sin(lamb);
    double cos_lambda = std::cos(lamb);
    double sin_phi = std::sin(phi);
    double cos_phi = std::cos(phi);
    double x0 = (height + N)*cos_lambda * cos_phi;
    double y0 = (height + N)*cos_lambda * sin_phi;
    double z0 = (height +(1- e_sq)* N)* sin_lambda;
    double t = cos_lambda *z_up - sin_lambda *y_north;
    double zd = sin_lambda *z_up+cos_lambda*y_north;
    double xd = cos_phi *t-sin_phi*x_east;
    double yd = sin_phi*t + cos_phi*x_east;

    double x= xd+x0;
    double y = yd+y0;
    double z = zd+z0;
    double x2 = std::pow(x,2);
    double y2 = std::pow(y,2);
    double z2 = std::pow(z,2);
    double e = std::sqrt(1-std::pow((b/a),2));
    double b2 = b*b;
    double e2 = e*e;
    double ep = e*(a/b);
    double r = std::sqrt(x2+y2);
    double r2 = r*r;
    double E2 = a*a - b*b;
    double F = 54*b2*z2;
    double G = r2+(1-e2)*z2 - e2*E2;
    double c=(e2*e2*F*r2)/(G*G*G);
	double s = std::pow((1+c+std::sqrt(c*c+2*c)),(1/3));
    double P = F/(3*std::pow((s+1/s+1),2)*G*G);
	double Q = std::sqrt(1+2*e2*e2*P);
    double ro = -(P*e2*r)/(1+Q) + std::sqrt((a*a/2)*(1+1/Q)-(P*(1-e2)*z2)/(Q*(1+Q))-P*r2/2);
    double tmp = std::pow((r-e2*ro),2);
    double U = std::sqrt(tmp + z2);
    double V = std::sqrt(tmp +(1-e2)*z2);
    double zo = (b2*z)/(a*V);
    double high = U*(1-b2/(a*V));
	double lat = std::atan((z+ep*ep*zo)/r);
    double temp = std::atan(y/x);
    double lon =temp-pi;
    if(x>=0){
            lon = temp;
    }
    else if(x<0 && y>=0){
            lon = pi + temp;
    }
	this->gps_lat = lat/(pi/180);
	this->gps_lon = lon/(pi/180);
	this->gps_high = high;

    result[0] = this->gps_lat;
    result[1] = this->gps_lon;
    result[2] = this->gps_high;


}*/


void gps_transform::get_ENU(double *msg){
	msg[0] = x_enu;
	msg[1] = y_enu;
	msg[2] = z_enu;
}

void gps_transform::get_home_ECEF(double *msg){
	msg[0] = home_ecef_x;
	msg[1] = home_ecef_y;
	msg[2] = home_ecef_z;
//	cout << "ECEF home(new) \tx:" << home_ecef_x <<",\ty: " << home_ecef_y <<",\tz: " << home_ecef_z << endl;

}
void gps_transform::get_ECEF(double *msg){
	msg[0] = X;
	msg[1] = Y;
	msg[2] = Z;
//	cout << "ECEF(new) \tx:" << X <<",\ty: " << Y <<",\tz: " << Z << endl;

}
/*void gps_transform::get_result(double *msg){
	msg[0] = gps_lat;
	msg[1] = gps_lon;
	msg[2] = gps_high;
}*/

