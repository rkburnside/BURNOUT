//Header file variable
#define MM

#define WAYPOINT_COUNT 19
#define WAYPOINTS_STRING \
int excel_waypoints[19][2] = {{17,932}, {-2194,1000}, {-2190,-119}, {-75,-250}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}};

#ifdef MM
//WAYPOINT AND SPEED PARAMETERS
#define WAYPOINT_ACCEPT 21	//waypoint acceptance radius in inches
#define S1 1500				// some default values:
#define S2 1560				//S1 1500, S2 1540, S3 1560, S4 1600, S5 1650, SB 1300
#define S3 1580				//This is the speed for negotiating wp's 
#define S4 2000 			//1680 is pretty ridiculously fast. Don't use for general use. maybe try 1650, 1720 fastest
#define SB 1250				//breaking speed default 1300
#define P1 50				//proximity to allow car to align with next waypoint in inches
#define P2 100				//close proximity to waypoint in inches
#define P3 625				//far proximity to waypoint in inches
#define BREAKING_SPEED 4000	//microseconds should be slightly faster than S3 so that the car slows down to S3 and continues at that speed default 6000
#define L1 5400
#define L2 20000
#define L3 200
#define L4 350
#define NULL_FF -30

//SENSOR PARAMETERS
//#define GYRO_CAL 470868410	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_CAL 235434205	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define STEER_ADJUST 1425	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 3500		// proportional gain, default it 4.0
#define CP_GAIN 0		//cross product gain. if steering is 
#define CLICK_INCHES 0.852	//used to determine the number of inches per click

//FIXED PARAMETERS
#define CAR_NAME "***MINUTEMAN***" //car name
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define TOGGLE 6			//digital pin for autopilot switch
#define CLICK_MAX 3			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define WP_SIZE 20 			//number of bytes for each waypoint
#endif

#ifdef RR
//WAYPOINT AND SPEED PARAMETERS
#define WAYPOINT_ACCEPT 53	//waypoint acceptance radius in inches
#define S1 1500				//stationary speed
#define S2 1600				//1650 is a creeping speed
#define S3 1700				//This is the speed for negotiating wp's 
#define S4 1700				//1800 is pretty ridiculously fast. Don't use for general use.
#define SB 1700				//breaking. adjust this parameter to allow creeping up on waypoints
#define P1 100				//proximity to allow car to align with next waypoint in inches
#define P2 100				//close proximity to waypoint in inches
#define P3 200				//far proximity to waypoint in inches
#define BREAKING_SPEED 7000	//microseconds should be slightly faster than S3 so that the car slows down to S3 and continues at that speed
#define L1 5000
#define L2 20000
#define L3 200
#define L4 350
#define NULL_FF 48

//SENSOR PARAMETERS
//#define GYRO_CAL 466186233	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_CAL 233093117	//234044150
#define STEER_ADJUST 1480	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 4000		//proportional gain, if navigation gets unstable, reduce.
#define CP_GAIN 0		//cross product gain. if steering is 
#define CLICK_INCHES 2.09380	//85 clicks in 25 feet (300 inches)

//FIXED PARAMETERS
#define CAR_NAME "***ROADRUNNER***" //car name
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define TOGGLE 6			//digital pin for autopilot switch
#define CLICK_MAX 1			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define WP_SIZE 20 			//number of bytes for each waypoint
#endif

/* ROADRUNNER
1500 - stop
1550 - slow
1600 - medium, but still slow
1650 - medium
1700 - medium fast
1800 - VERY fast - spinouts occur
2000 - OOC (out of control) - LOTS of spinouts, dangerous

Millisecond speeds
1600 - 15000 ~ 17000 (usually 15000)
1650 - 12000
1700 - 9000
1750 - 5400
1800 - 6100

*/
