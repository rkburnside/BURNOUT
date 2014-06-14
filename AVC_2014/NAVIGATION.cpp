//NAVIGATION FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
int steer_us;
double x_wp = 0, y_wp = 0;
double target_x=0, target_y=0;
double angle_last, angle_target, x=0, y=0;
static int steer_limm = 200;
static double cross_product=0;
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
	if(proximity < (WAYPOINT_ACCEPT/CLICK_INCHES)){
		wpr_count++;
		EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
		x_wp = waypoint.x;
		y_wp = waypoint.y;
		if (((int)x_wp == 0) && ((int)y_wp == 0)) end_run(); // 0,0 is interpreted as the final waypoint. end run.
		Serial.print("read WP #");
		Serial.print(wpr_count);
		Serial.print(": ");
		Serial.print(x_wp*CLICK_INCHES);
		Serial.print(" , ");
		Serial.println(y_wp*CLICK_INCHES);
		double temp = pow((x_wp-x),2);
		temp += pow((y_wp-y),2);
		proximity = sqrt(temp);
		//proximity = sqrt(pow((x_wp - x),2) + pow((y_wp - y),2));	
		previous_proximity = proximity;

		//sets up the planned cross product target vectors
		target_x = x_wp - target_x;
		target_y = y_wp - target_y;
	}
	
	return ;
}

void update_position(){
	//calculate position
	x += sin(angle);
	y += cos(angle);
	angle_target = atan2((x_wp - x),(y_wp - y));
	double temp = pow((x_wp-x),2);
	temp += pow((y_wp-y),2);
	proximity = sqrt(temp);
	//proximity = sqrt(pow((x_wp - x),2) + pow((y_wp - y),2));	
	return ;
}

void cal_steer_lim(){
	steer_limm = (int)map(speed_cur, L1, L2, L3, L4);
	if(steer_limm > L4) steer_limm = L4;
	
	return ;
}

void speed(){
	running = true;			// make sure running is updated.

	if((previous_proximity - proximity) <= (P1/CLICK_INCHES)) esc.writeMicroseconds(S2); //allow car to line up with the next point
	else if(proximity < (P2/CLICK_INCHES)) esc.writeMicroseconds(S2); //ensure that a waypoint can be accepted
	else if(proximity >= (P2/CLICK_INCHES) && proximity < (P3/CLICK_INCHES)){ //slow way down  50-200 works well, 50-300 is more conservative for higher speeds
		if(speed_cur < BREAKING_SPEED)  esc.writeMicroseconds(SB);  // less than 8000 means high speed, apply brakes
		else esc.writeMicroseconds(S3);  //once speed is low enough, resume normal slow-down
	}
	else if(proximity >= (P3/CLICK_INCHES)) esc.writeMicroseconds(S4); //go wide open 200 works well for me. 

	return ;
}

void update_steering(){
	// calculate and write angles for steering
	angle_diff = angle_target - angle;
	if(angle_diff < -3.14159) angle_diff += 3.14159*2;   //if angle is less than 180 deg, then add 360 deg
	if(angle_diff > 3.14159) angle_diff -= 3.14159*2;	//if angle is greater than 180 deg, then subtract 360
	// now, we have an angle as -180 < angle_diff < 180.
	// steer_us = angle_diff/(3.14159*2.0)*STEER_GAIN;
	steer_us = angle_diff/(3.14159*2.0)*STEER_GAIN;	//cross product gain added  here so that the steering is still limited
	//steer_us = 0 - steer_us;
	if(steer_us < (0-steer_limm)) steer_us = 0-steer_limm;
	if(steer_us > steer_limm) steer_us = steer_limm;
	//steer_us = 0;
	steer_us += STEER_ADJUST;  //adjusts steering so that it will go in a straight line
	//Serial.println(steer_us);
	return ;
}

void update_cross_product(){
	//calculates the car's current vector
	double current_x = x_wp - x;
	double current_y = y_wp - y;

	//determines magnitude of each vector
	double mag_target = sqrt(target_x*target_x + target_y*target_y);
	if(mag_target == 0.0) mag_target = 0.0001;
	double mag_current = sqrt(current_x*current_x + current_y*current_y);
	if(mag_current == 0.0) mag_current = 0.0001;
	
	//make the current and target vectors into unit vectors
	target_x = target_x / mag_target;
	target_y = target_y / mag_target;
	current_x = current_x / mag_current;
	current_y = current_y / mag_current;
	
	//the actual cross product calculation
	cross_product = -(target_x*current_y - current_x*target_y);
	
	return ;
}

void end_run() {    // go straight forward, slowly at last waypoint
	esc.writeMicroseconds(S2); //reduce speed
	steer_us = STEER_ADJUST;  // go straight
	while(true); //loop endlessly
}

void calculate_speed(){
    speed_new = micros();
    speed_cur = speed_new - speed_old;
    speed_old = speed_new;
	
	return ;
}

void print_coordinates(){ //print target, location, etc.
	Serial.print("(x,y): ");
	Serial.print(x*CLICK_INCHES);
	Serial.print(" , ");
	Serial.print(y*CLICK_INCHES);	Serial.print("   trgt: ");
	Serial.print(x_wp*CLICK_INCHES);
	Serial.print(" , ");
	Serial.println(y_wp*CLICK_INCHES);

	return ;
}
