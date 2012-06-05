//Header file variable
#define MM

#define WAYPOINT_COUNT 19
#define WAYPOINTS_STRING \
double excel_waypoints[19][2] = {{0, 0}, {-70.68, 203.29}, {-194.87, 464}, {-207, 842}, {-70, 897}, {-25, 669}, {-5, 148}, {0, 4000}, {0, 4500}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};

#ifdef MM
#define GYRO_CAL 8650000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 1.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1425	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***MINUTEMAN***" //car name
#define WAYPOINT_ACCEPT 25	//waypoint acceptance radius

#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations default 1000
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define CLICK_MAX 3			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 3500		// proportional gain, default it 4.0
#define WP_SIZE 20 			//number of bytes for each waypoint
#define S1 1500				// some default values:
#define S2 1560				// S1 1500, S2 1540, S3 1560, S4 1600, S5 1650, SB 1300
#define S3 1580				// This is the speed for negotiating wp's 
#define S4 1650				// 1680 is pretty ridiculously fast. Don't use for general use. maybe try 1650
#define S5 1650
#define SB 1300
#endif

#ifdef RR
#define GYRO_CAL 8700000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 0.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1475	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***ROADRUNNER***" //car name
#define WAYPOINT_ACCEPT 100	//waypoint acceptance radius

#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations default 1000
#define MODE 5				//digital pin for mode select, default 5
#define TMISO 4				//digital pin for autopilot enable/disable, default 4
#define CLICK_MAX 3			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed, default 300
#define STEER_GAIN 4000		// proportional gain, if navigation gets unstable, reduce.
#define WP_SIZE 20 			//number of bytes for each waypoint
#define S1 1500
#define S2 1625
#define S3 1650
#define S4 1750

#endif

/*
0%	1500	20%	1600	40%	1700	60%	1800	80%	1900	100% 2000
5%	1525	25%	1625	45%	1725	65%	1825	85%	1925
10%	1550	30%	1650	50%	1750	70%	1850	90%	1950
15%	1575	35%	1675	55%	1775	75%	1875	95%	1975
*/


