//Header file variable
#define SERVO 2						// pin # for steering servo - green	
#define THROTTLE 3					// pin # for throttle - yellow wire
#define WAYPOINT_ACCEPT_RANGE 10 	// radius in # of feet in which to accept the waypoint
#define COMPASS_X_CAL -92			// compass calibration number
#define COMPASS_Y_CAL 157			// compass calibration number
#define DECLINATION 0.15126187		// compass declination number

//GPS Variables
float flat, flon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
double waypoint_distance, waypoint_heading = 0;

//Compass
int XAxis = 0, YAxis = 0;
double compass_heading = 0;
double angle_diff;				// for the compass


//GPS Waypoints
int waypoint_num = 0;
const int waypoint_total = 5;	// <- should always be the same number of GPS waypoints
// 0 = lat use, 1 = south lat use, 2 = north lat use
double gps_array[5][3] = {{39.538696506815334, -105.01680727005721, 0},
{39.53873270565525, -105.01672948599578, 0},
{39.53847827912335, -105.01641834975005, 0},
{39.53861066377666, -105.01659805775405, 0},
{39.53863031460211, -105.01675362587692, 0}};

double print_delay = 0;













/*

#define WAYPOINT_COUNT 19
#define WAYPOINTS_STRING \
double excel_waypoints[19][2] = {{0, 1000}, {546.09, 1360}, {2200.11, 1300}, {3832, 1190}, {3760, -1276}, {1246, -1430}, {160.34, -1250}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}, {214.93, 274.15}};

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

//SENSOR PARAMETERS
#define GYRO_CAL 8700000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 0.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1475	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 4000		// proportional gain, if navigation gets unstable, reduce.

//FIXED PARAMETERS
#define CAR_NAME "***ROADRUNNER***" //car name
#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations default 1000
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define CLICK_MAX 3			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define WP_SIZE 20 			//number of bytes for each waypoint

*/