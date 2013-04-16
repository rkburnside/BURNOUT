//Header file variable
int compass_x_cal = -92;			//compass calibration number
int compass_y_cal = 157;			//compass calibration number
#define DECLINATION 0.15126187		//compass declination number

#define WAYPOINT_ACCEPT_RANGE 10 	//radius in # of feet in which to accept the waypoint

#define SERVO 2						//pin # for steering servo - green	
#define SERVO_STEERING_LIMIT_LEFT 150
#define SERVO_STEERING_LIMIT_RIGHT 30

#define THROTTLE 3					//pin # for throttle - yellow wire
#define SPEED_STOP 90
#define SPEED_SLOW 100
#define SPEED_MED 105
#define SPEED_FAST 120
#define SPEED_BREAKING 50
#define ERROR_GAIN 1.0				//1.0 full gain, .5 half as aggressive, .25 quarter aggressive

//Global Variables
double print_delay = 0;
double cross_track_error = 0;

//GPS Variables
float flat, flon, max_speed=0.0;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
double waypoint_distance, waypoint_heading = 0.0;

//Compass
int XAxis = 0, YAxis = 0;
double compass_heading = 0;
double angle_diff;				//for the compass

//GPS Waypoints
int waypoint_num = 0;
const int waypoint_total = 5;	//<- should always be the same number of GPS waypoints
double gps_array[5][2] = {{39.538696506815334, -105.01680727005721},
{39.53873270565525, -105.01672948599578},
{39.53847827912335, -105.01641834975005},
{39.53861066377666, -105.01659805775405},
{39.53863031460211, -105.01675362587692}};


/*
//WAYPOINT AND SPEED PARAMETERS
#define WAYPOINT_ACCEPT 50	//waypoint acceptance radius
#define S1 1550				//stationary speed
#define S2 1635				//1650 is a creeping speed
#define S3 1675				//This is the speed for negotiating wp's 
#define S4 1750				//1800 is pretty ridiculously fast. Don't use for general use.
#define SB 1050				//breaking. adjust this parameter to allow creeping up on waypoints
#define P1 150				//proximity to allow car to align with next waypoint 
#define P2 75				//close proximity to waypoint
#define P3 300				//far proximity to waypoint
#define BREAKING_SPEED 3000	//microseconds should be slightly faster than S3 so that the car slows down to S3 and continues at that speed
#define L1 2200
#define L2 5500
#define L3 200
#define L4 350
*/