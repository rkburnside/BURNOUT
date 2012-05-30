//Header file variable
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

#endif

#ifdef RR
#define GYRO_CAL 8700000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 0.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1515	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***ROADRUNNER***" //car name
#define WAYPOINT_ACCEPT 70	//waypoint acceptance radius

#define DEBUG 0				//debug state  1=cal gyro, 2=watch angle, 3=read waypoints
#define GYRO_LIMIT 1000		//defines how many gyro samples are taken between angle calculations
#define MODE 5				//digital pin for mode select
#define TMISO 4				//digital pin for autopilot enable/disable
#define CLICK_MAX 3		//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position
#define SERVO_LIM 300		//limits the swing of the servo so it does not get overstressed
#define WP_SIZE 20 			//number of bytes for each waypoint

#endif
