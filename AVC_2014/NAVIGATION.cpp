//NAVIGATION FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
int steer_us;
double x_wp = 0, y_wp = 0, x_wp0 = 0, y_wp0 = 0;
double target_x=0, target_y=0;
double angle_last, angle_target, angle_vtp, x=0, y=0, speed_mph;
static int steer_limm = SERVO_LIM;

static int wp_accept = 50;
//static double cross_product=0;
static double angle_diff;
static long speed_cur=0, speed_new=0, speed_old=0;
static long proximity, previous_proximity=50;


//EXTERNAL VARIABLES
extern double angle;
extern bool running;
extern byte wpr_count;


//OBJECT DECLARATIONS
extern Servo steering, esc;
extern position_structure waypoint;


//PROGRAM FUNCTIONS
void update_waypoint(){
	//waypoint acceptance and move to next waypoint
	if(proximity < (wp_accept)){
		x_wp0 = x_wp;
		y_wp0 = y_wp;
		wpr_count++;
		EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
		x_wp = waypoint.x;
		y_wp = waypoint.y;
		SERIAL_OUT.print("#");
		SERIAL_OUT.print(wpr_count);
		SERIAL_OUT.print(",");
		SERIAL_OUT.print(x_wp);
		SERIAL_OUT.print(",");
		SERIAL_OUT.println(y_wp);
		double temp = pow((x_wp-x),2);
		temp += pow((y_wp-y),2);
		proximity = sqrt(temp);
		//proximity = sqrt(pow((x_wp - x),2) + pow((y_wp - y),2));	
		previous_proximity = proximity;
	}
	
	return ;
}


// void map_rates(){
// /*
// the following should depend on current speed:
// max turn rate, steering gain, acceptance radius, lookahead distance
// */
// int temp = speed_mph;
// if ((int)speed_mph < 10) temp = 10;
// steer_limm = map(temp, 10, 35, 200, 350);

// temp = speed_mph; //if ((int)speed_mph 15) temp = 15;
// steer_gain = map(temp, 15, 35, 4000, 3500);

// temp = speed_mph; //if ((int)speed_mph 15) temp = 15;
// waypoint_accept = map(temp, 15, 35, 21, 50);

// temp = speed_mph; //if ((int)speed_mph 5) temp = 5;
// look_ahead = map(temp, 5, 35, 20, 100);

// }

void update_position(){
	//calculate position
	x += CLICK_INCHES * sin(angle);
	y += CLICK_INCHES * cos(angle);
	angle_target = atan2((x_wp - x),(y_wp - y));
	double temp = pow((x_wp-x),2);
	temp += pow((y_wp-y),2);
	proximity = sqrt(temp);
	//proximity = sqrt(pow((x_wp - x),2) + pow((y_wp - y),2));	
	return ;
}

void cal_steer_lim(){
	steer_limm = map((int)speed_mph, L1, L2, L3, L4);
	//if(steer_limm > L4) steer_limm = L4;
	//steer_limm = 250;		//********************** cancel this for normal operation !!!!!!!!!!!!!!!!*********************
	
	return ;
}

void cal_wp_accept(){
	wp_accept = map((int)speed_mph, L1, L2, L3, L4);
	return ;
}

void speed(){
	running = true;			// make sure running is updated.
	angle_diff = angle_diff * 180.0/3.14159;
	angle_diff = abs(angle_diff);
	if (angle_diff < SPEED_TOGGLE_ANGLE)  esc.writeMicroseconds(S4);
	else esc.writeMicroseconds(S2);
	return ;
}

void update_steering(){
	// calculate and write angles for steering
	angle_diff = angle_target - angle;
	if (PATH_FOLLOWING) angle_diff = angle_vtp - angle;
	if(angle_diff < -3.14159) angle_diff += 6.2831;   //if angle is less than 180 deg, then add 360 deg
	if(angle_diff > 3.14159) angle_diff -= 6.2831;	//if angle is greater than 180 deg, then subtract 360
	// now, we have an angle as -180 < angle_diff < 180.
	// steer_us = angle_diff/(3.14159*2.0)*STEER_GAIN;
	steer_us = (int)(angle_diff*STEER_GAIN);	//
	if(steer_us < (0-steer_limm)) steer_us = 0-steer_limm;
	if(steer_us > steer_limm) steer_us = steer_limm;
	steer_us += STEER_ADJUST;  //adjusts steering so that it will go in a straight line
	return ;
}

void calculate_speed(){
	speed_new = micros();
	speed_cur = speed_new - speed_old;
	speed_old = speed_new;
	speed_mph = CLICK_INCHES * 56818.0 / speed_cur;
	//SERIAL_OUT.println(speed_cur);
	//cal_steer_lim();
	cal_wp_accept();
	return ;
}

void calculate_look_ahead(){
	//int time = micros();
	double Ru = sqrt(pow(x - x_wp0,2) + pow(y - y_wp0,2));
	double theta = atan2(y_wp - y_wp0, x_wp - x_wp0);
	double theta_u = atan2(y - y_wp0, x - x_wp0);
	double beta = theta - theta_u;
	double R = sqrt(pow(Ru,2) - pow(Ru*sin(beta),2));
	double x_vtp = (R+LOOK_AHEAD)*cos(theta) +x_wp0;
	double y_vtp = (R+LOOK_AHEAD)*sin(theta) +y_wp0;
	angle_vtp = atan2((x_vtp - x), (y_vtp - y));
}

/*
print x, y, speed, angle, target_angle, gyro rate, accel rate, proximity, 
use the following commands:
		temp = accelgyro.getRotationZ();
		temp = accelgyro.getAccelerationY();

stuff to print on first time through
steer_gain, steer_lim, look_ahead, target speed, 
		
		
print either to serial or print using the radio module
will need separate routines for each. enable with ifdef

print to radio ideas:
it looks like the payload for each packet is 32 bytes. probably have to split up in 2 or 3 packets
probably use PString library

On the reciever side, simply wait for packets, and write them to the serial port as they arive. 



*/
void print_parameters(){
	SERIAL_OUT.print('p');
	SERIAL_OUT.print(wp_accept); SERIAL_OUT.print(',');
	SERIAL_OUT.print(S1); SERIAL_OUT.print(',');
	SERIAL_OUT.print(S2); SERIAL_OUT.print(',');
	SERIAL_OUT.print(S3); SERIAL_OUT.print(',');
	SERIAL_OUT.print(S4); SERIAL_OUT.print(',');
	SERIAL_OUT.print(SB); SERIAL_OUT.print(',');
	SERIAL_OUT.print(P1); SERIAL_OUT.print(',');
	SERIAL_OUT.print(P2); SERIAL_OUT.print(',');
	SERIAL_OUT.print(P3); SERIAL_OUT.print(',');
	SERIAL_OUT.print(BREAKING_SPEED); SERIAL_OUT.print(',');
	SERIAL_OUT.print(L1); SERIAL_OUT.print(',');
	SERIAL_OUT.print(L2); SERIAL_OUT.print(',');
	SERIAL_OUT.print(L3); SERIAL_OUT.print(',');
	SERIAL_OUT.print(L4); SERIAL_OUT.print(',');
	SERIAL_OUT.print(PATH_FOLLOWING); SERIAL_OUT.print(',');
	SERIAL_OUT.print(LOOK_AHEAD); SERIAL_OUT.print(',');
	SERIAL_OUT.print(GYRO_CAL); SERIAL_OUT.print(',');
	SERIAL_OUT.print(STEER_ADJUST); SERIAL_OUT.print(',');
	SERIAL_OUT.print(SERVO_LIM); SERIAL_OUT.print(',');
	SERIAL_OUT.print(STEER_GAIN); SERIAL_OUT.print(',');
	SERIAL_OUT.print(SPEED_TOGGLE_ANGLE); SERIAL_OUT.print(',');
	SERIAL_OUT.println(CLICK_INCHES);
	
	return ;
}

void print_data(){
	SERIAL_OUT.print('d');
	SERIAL_OUT.print(x); SERIAL_OUT.print(',');
	SERIAL_OUT.print(y); SERIAL_OUT.print(',');
	SERIAL_OUT.print(speed_mph); SERIAL_OUT.print(',');
	SERIAL_OUT.print(get_gyro_rate()); SERIAL_OUT.print(',');
	SERIAL_OUT.print(get_accel_rate()); SERIAL_OUT.print(',');
	SERIAL_OUT.print(steer_us); SERIAL_OUT.print(',');
	SERIAL_OUT.print(angle); SERIAL_OUT.print(',');
	SERIAL_OUT.print(proximity); SERIAL_OUT.print(',');
	SERIAL_OUT.println(angle_diff);

}

void print_coordinates(){ //print target, location, etc.
	SERIAL_OUT.print("(x,y): ");
	SERIAL_OUT.print(x);
	SERIAL_OUT.print(" , ");
	SERIAL_OUT.print(y);	SERIAL_OUT.print("   trgt: ");
	SERIAL_OUT.print(x_wp);
	SERIAL_OUT.print(" , ");
	SERIAL_OUT.print(y_wp);
	SERIAL_OUT.print(" , ");
	SERIAL_OUT.println(steer_limm);
//	SERIAL_OUT.println(angle_diff);

	return ;
}
