//Header file variable
#define RR

#define WAYPOINT_COUNT 19
#define WAYPOINTS_STRING \
//int excel_waypoints[5][2] = {{0, 0}, {0, 250}, {0, 500}, {0, 750}, {0, 1000}};

int excel_waypoints[19][2] = {{0, 0}, {0, 250}, {0, 500}, {0, 750}, {0, 1000}, {0, 1250}, {0, 1500}, {0, 1750}, {0, 2000}, {0, 2250}, {0, 2500}, {0, 2750}, {0, 3000}, {0, 3250}, {0, 3500}, {0, 3750}, {0, 4000}, {0, 4250}, {0, 4500}};

#ifdef MM
#define GYRO_CAL 8650000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 1.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1425	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***MINUTEMAN***" //car name
#define WAYPOINT_ACCEPT 25	//waypoint acceptance radius

#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations
#define MODE 5				//digital pin for mode select
#define TMISO 4				//digital pin for autopilot enable/disable
#define CLICK_MAX 3		//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed
#define WP_SIZE 20 			//number of bytes for each waypoint
#define S1 1500
#define S2 1540
#define S3 1540
#define S4 1560
#define S5 2560

#endif

#ifdef RR
#define GYRO_CAL 8700000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 0.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1475	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***ROADRUNNER***" //car name
#define WAYPOINT_ACCEPT 250	//waypoint acceptance radius

#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations
#define MODE 5				//digital pin for mode select
#define TMISO 4				//digital pin for autopilot enable/disable
#define CLICK_MAX 3		//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed
#define WP_SIZE 20 			//number of bytes for each waypoint
#define S1 1580
#define S2 1600
#define S3 1650
#define S4 1700
#define S5 1750

#endif

/*
0%	1500	20%	1600	40%	1700	60%	1800	80%	1900	100% 2000
5%	1525	25%	1625	45%	1725	65%	1825	85%	1925
10%	1550	30%	1650	50%	1750	70%	1850	90%	1950
15%	1575	35%	1675	55%	1775	75%	1875	95%	1975
*/


