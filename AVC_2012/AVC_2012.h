//Header file variable
#ifdef MM
#define GYRO_CAL 8650000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 1.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1425	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***MINUTEMAN***" //car name
#define WAYPOINT_ACCEPT 25	//waypoint acceptance radius
#endif

#ifdef RR
#define GYRO_CAL 8700000	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define TIRE_CAL 1.5		//tire calibration factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define STEER_ADJUST 1515	//steering adjustment factor. ***THIS IS JUST A PLACE HOLDER FOR NOW***
#define CAR_NAME "***ROADRUNNER***" //car name
#define WAYPOINT_ACCEPT 70	//waypoint acceptance radius
#endif
