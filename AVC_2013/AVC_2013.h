//Header file variable
#define MM

#define WAYPOINT_COUNT 19
#define WAYPOINTS_STRING \
int excel_waypoints[19][2] = {{24.07,131.26}, {-374.59,200.95}, {-502.73,176.66}, {-613.58,232.41}, {-864.93,248.96}, {-1058.33,240}, {-1058.33,130.61}, {-1011.68,-51.44}, {-658.5,-116.11}, {-545.21,-128.71}, {26.41,-154.48}, {47.48,-35.67}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}};

#ifdef MM
//WAYPOINT AND SPEED PARAMETERS
#define WAYPOINT_ACCEPT 10	//waypoint acceptance radius
#define S1 1500				// some default values:
#define S2 1560				//S1 1500, S2 1540, S3 1560, S4 1600, S5 1650, SB 1300
#define S3 1580				//This is the speed for negotiating wp's 
#define S4 2000 				//1680 is pretty ridiculously fast. Don't use for general use. maybe try 1650, 1720 fastest
#define SB 1250				//breaking speed default 1300
#define P1 25				//proximity to allow car to align with next waypoint 
#define P2 50				//close proximity to waypoint
#define P3 300				//far proximity to waypoint default 250
#define BREAKING_SPEED 4000	//microseconds should be slightly faster than S3 so that the car slows down to S3 and continues at that speed default 6000
#define L1 5400
#define L2 20000
#define L3 200
#define L4 350
#define NULL_FF -30

//SENSOR PARAMETERS
//#define GYRO_CAL 470868410	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_CAL 235434205	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 1.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1425	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 3500		// proportional gain, default it 4.0
#define CP_GAIN 250		//cross product gain. if steering is 

//FIXED PARAMETERS
#define CAR_NAME "***MINUTEMAN***" //car name
#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations default 1000
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define CLICK_MAX 3			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define WP_SIZE 20 			//number of bytes for each waypoint
#endif

#ifdef RR
//WAYPOINT AND SPEED PARAMETERS
#define WAYPOINT_ACCEPT 25	//waypoint acceptance radius
#define S1 1500				//stationary speed
#define S2 1600				//1650 is a creeping speed
#define S3 1700				//This is the speed for negotiating wp's 
#define S4 1750				//1800 is pretty ridiculously fast. Don't use for general use.
#define SB 1250				//breaking. adjust this parameter to allow creeping up on waypoints
#define P1 50				//proximity to allow car to align with next waypoint 
#define P2 50				//close proximity to waypoint
#define P3 300				//far proximity to waypoint
#define BREAKING_SPEED 3000	//microseconds should be slightly faster than S3 so that the car slows down to S3 and continues at that speed
#define L1 5000
#define L2 20000
#define L3 200
#define L4 350
#define NULL_FF 48

//SENSOR PARAMETERS
//#define GYRO_CAL 466186233	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_CAL 233093117	//234044150
#define TIRE_CAL 1.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1480	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 4000		//proportional gain, if navigation gets unstable, reduce.
#define CP_GAIN 250		//cross product gain. if steering is 

//FIXED PARAMETERS
#define CAR_NAME "***ROADRUNNER***" //car name
#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations default 1000
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define CLICK_MAX 1			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define WP_SIZE 20 			//number of bytes for each waypoint
#endif

/*
1500 - stop
1550 - slow
1600 - medium, but still slow
1650 - medium
1700 - medium fast
1800 - VERY fast - spinouts occur
2000 - OOC - LOTS of spinouts, out of control
*/

/*
0%	1500	20%	1600	40%	1700	60%	1800	80%	1900	100% 2000
5%	1525	25%	1625	45%	1725	65%	1825	85%	1925
10%	1550	30%	2311650	50%	1750	70%	1850	90%	1950
15%	1575	35%	1675	55%	1775	75%	1875	95%	1975
*/


