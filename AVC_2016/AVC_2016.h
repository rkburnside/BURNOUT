//AVC SETTINGS
/* SET UP PROCESS
set vehicle name
set set communication method
set waypoints from a file!!!
set starting position x
set starting position y
upload the code
*/

#define TAM			//use either MM (minuteman) or RR (roadrunner) or WC (Wile E. Coyote)
#define BLUETOOTH 	//use either BLUETOOTH or USB to define the serial port for program output

#define WAYPOINT_COUNT 29
#define WAYPOINTS_STRING \
int excel_waypoints[30][2] = {{36,293}, {572,329}, {638,244}, {649,-111}, {1000,-186}, {1065,-257}, {1013,-1034}, {845,-1074}, {715,-761}, {818,-465}, {671,-389}, {390,-720}, {275,-900}, {175,-1025}, {83,-1042}, {-12,-980}, {64,50}, {0,1}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}, {0,250}};
#define STARTING_POSITION_X 0.0
#define STARTING_POSITION_Y 0.0

#ifdef TIT
//WAYPOINT AND SPEED PARAMETERS
//WAYPOINT PARAMETERS
#define WAYPOINT_ACCEPT 60	//waypoint acceptance radius: speed = 11650 -> 48, speed = 1850 -> 180

//SPEED PARAMETERS
#define S_STOP 1500				//stationary speed
#define S_LOW 1575				//slow speed: 1600 ~ 1650
#define S_HIGH 1575				//top speed: 1650 ~ 1800
#define FAST 1575				//FAST start off the line ONLY

//STEERING PARAMETERS
#define L1 5000
#define L2 20000
#define L3 200
#define L4 350
#define STEER_ADJUST 1500			//steering adjustment factor
#define STEER_GAIN -1000				//proportional gain, 4000 = servo slammed, 2000 = servo slams less, 300 = supposedly OK
#define PATH_FOLLOWING 0
#define LOOK_AHEAD 100				//120 = too high, 20 = too low, 50 = aggressive, but good, 80 = not aggressive, but OK
#define SPEED_TOGGLE_ANGLE 15.0		//20 +/- 5 degrees is good

//GYRO PARAMETERS
#define GYRO_CAL 232568254			//this has to be measured by rotating the gyro 360 deg. and reading the output and then dividing by the number of rotations and by 2 to get a 180-deg number
#define XGYROOFFSET -861
#define YGYROOFFSET -9
#define ZGYROOFFSET 30

//MISCELLANEOUS PARAMETERS
#define CAR_NAME "***TITAN***" //car name
#define WP_SIZE 20 					//number of bytes for each waypoint
#define CLICK_INCHES 4.137931		//145 clicks in 25 feet (300 inches) = 2.0689655, it is now doubled because the interrupt is set to rising instead of change
#define CLICK_MAX 2			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3

//Teensy Pin Assignments:
//Optional manual throttle, autonomous steering - connect ESC signal pin to receiver CH2
#define RESET_PIN 2
#define MODE_LINE_1 5
#define MODE_LINE_2 6
#define THROTTLE 20
#define STEERING 21
#define HALL_EFFECT_SENSOR 22
#define TOGGLE 23

//CH3 settings
#define MANUAL 0
#define AUTOMATIC 1
#define WP_MODE 2
#define AUX 3
#define RESET 4
#endif

#ifdef RR		//(1675 = ~42 second run, 38 seconds on a full charge)
//WAYPOINT PARAMETERS
#define WAYPOINT_ACCEPT 80	//waypoint acceptance radius: speed = 11650 -> 48, speed = 1850 -> 180

//SPEED PARAMETERS

//50 second speed
#define S_STOP 1500				//stationary speed
#define S_LOW 1625				//slow speed: 1600 ~ 1650
#define S_HIGH 1635				//top speed: 1650 ~ 1800
#define FAST 1725				//FAST start off the line ONLY

// 45 second speed
// #define S_STOP 1500				//stationary speed
// #define S_LOW 1625				//slow speed: 1600 ~ 1650
// #define S_HIGH 1650				//top speed: 1650 ~ 1800
// #define FAST 1850				//FAST start off the line ONLY

//40 second speed
//#define S_STOP 1500				//stationary speed
//#define S_LOW 1625				//slow speed: 1600 ~ 1650
//#define S_HIGH 1675				//top speed: 1650 ~ 1800
// #define FAST 1850				//FAST start off the line ONLY

//37 second speed
//#define S_STOP 1500				//stationary speed
//#define S_LOW 1625				//slow speed: 1600 ~ 1650
//#define S_HIGH 1700				//top speed: 1650 ~ 1800
// #define FAST 1850				//FAST start off the line ONLY

//36 second speed
//#define S_STOP 1500				//stationary speed
//#define S_LOW 1625				//slow speed: 1600 ~ 1650
//#define S_HIGH 1750				//top speed: 1650 ~ 1800
// #define FAST 1850				//FAST start off the line ONLY

//STEERING PARAMETERS
#define L1 5000
#define L2 20000
#define L3 200
#define L4 350
#define STEER_ADJUST 1505			//steering adjustment factor
#define STEER_GAIN 318				//proportional gain, 4000 = servo slammed, 2000 = servo slams less, 300 = supposedly OK
#define PATH_FOLLOWING 1
#define LOOK_AHEAD 80				//120 = too high, 20 = too low, 50 = aggressive, but good, 80 = not aggressive, but OK
#define SPEED_TOGGLE_ANGLE 15.0		//20 +/- 5 degrees is good

//GYRO PARAMETERS
#define GYRO_CAL 233302330			//this has to be measured by rotating the gyro 360 deg. and reading the output and then dividing by the number of rotations and by 2 to get a 180-deg number
#define XGYROOFFSET -861
#define YGYROOFFSET -9
#define ZGYROOFFSET 30

//MISCELLANEOUS PARAMETERS
#define CAR_NAME "***ROADRUNNER***" //car name
#define WP_SIZE 20 					//number of bytes for each waypoint
#define CLICK_INCHES 4.137931		//145 clicks in 25 feet (300 inches) = 2.0689655, it is now doubled because the interrupt is set to rising instead of change
#define CLICK_MAX 2			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3

//Teensy Pin Assignments:
//Optional manual throttle, autonomous steering - connect ESC signal pin to receiver CH2
#define RESET_PIN 2
#define MODE_LINE_1 5
#define MODE_LINE_2 6
#define THROTTLE 20
#define STEERING 21
#define HALL_EFFECT_SENSOR 22
#define TOGGLE 23

//CH3 settings
#define MANUAL 0
#define AUTOMATIC 1
#define WP_MODE 2
#define AUX 3
#define RESET 4
#endif

#ifdef OB
//WAYPOINT PARAMETERS
#define WAYPOINT_ACCEPT 64		//waypoint acceptance radius: speed = 1650 -> 48, speed = 1850 -> 180

//SPEED PARAMETERS
#define S_STOP 1500				//stationary speed
#define S_LOW 2000				//slow speed: 1600 ~ 1625
#define S_HIGH 2000				//top speed: 1650 ~ 1800
#define FAST 2000				//FAST start off the line ONLY

//STEERING PARAMETERS
#define L1 5000
#define L2 20000
#define L3 100
#define L4 250
#define STEER_ADJUST 1425			//steering adjustment factor
#define STEER_GAIN 500			//proportional gain, 4000 = servo slammed, 2000 = servo slams less, 300 = supposedly OK
#define PATH_FOLLOWING 1
#define LOOK_AHEAD 80				//120 = too high, 20 = too low, 50 = aggressive, but good, 80 = not aggressive, but OK
#define SPEED_TOGGLE_ANGLE 20		//20 +/- 5 degrees is good

//GYRO PARAMETERS
#define GYRO_CAL 231719139			//this has to be measured by rotating the gyro 360 deg. and reading the output and then dividing by the number of rotations and by 2 to get a 180-deg number
#define XGYROOFFSET -30
#define YGYROOFFSET 32
#define ZGYROOFFSET -10

//MISCELLANEOUS PARAMETERS
#define CAR_NAME "***OVERBEARING***" //car name
#define WP_SIZE 20 				//number of bytes for each waypoint
#define CLICK_INCHES 3.0001	//69~70 clicks in 25 feet (300 inches)
#define CLICK_MAX 1				//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3

//Teensy Pin Assignments:
//Optional manual throttle, autonomous steering - connect ESC signal pin to receiver CH2
#define RESET_PIN 2
#define MODE_LINE_1 5
#define MODE_LINE_2 6
#define THROTTLE 20
#define STEERING 21
#define HALL_EFFECT_SENSOR 22
#define TOGGLE 23

//CH3 settings
#define MANUAL 0
#define AUTOMATIC 1
#define WP_MODE 2
#define AUX 3
#define RESET 4
#endif

#ifdef TAM
//WAYPOINT PARAMETERS
#define WAYPOINT_ACCEPT 64		//waypoint acceptance radius: speed = 1650 -> 48, speed = 1850 -> 180

//SPEED PARAMETERS
#define S_STOP 1500				//stationary speed
#define S_LOW 2000				//slow speed: 1600 ~ 1625
#define S_HIGH 2000				//top speed: 1650 ~ 1800
#define FAST 2000				//FAST start off the line ONLY

//STEERING PARAMETERS
#define L1 5000
#define L2 20000
#define L3 100
#define L4 250
#define STEER_ADJUST 1425			//steering adjustment factor
#define STEER_GAIN 500			//proportional gain, 4000 = servo slammed, 2000 = servo slams less, 300 = supposedly OK
#define PATH_FOLLOWING 1
#define LOOK_AHEAD 80				//120 = too high, 20 = too low, 50 = aggressive, but good, 80 = not aggressive, but OK
#define SPEED_TOGGLE_ANGLE 20		//20 +/- 5 degrees is good

//GYRO PARAMETERS
#define GYRO_CAL 231719139			//this has to be measured by rotating the gyro 360 deg. and reading the output and then dividing by the number of rotations and by 2 to get a 180-deg number
#define XGYROOFFSET -30
#define YGYROOFFSET 32
#define ZGYROOFFSET -10

//MISCELLANEOUS PARAMETERS
#define CAR_NAME "***TAMAHAWK***" //car name
#define WP_SIZE 20 				//number of bytes for each waypoint
#define CLICK_INCHES 3.0001	//69~70 clicks in 25 feet (300 inches)
#define CLICK_MAX 1				//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3

//Teensy Pin Assignments:
//Optional manual throttle, autonomous steering - connect ESC signal pin to receiver CH2
#define RESET_PIN 2
#define MODE_LINE_1 5
#define MODE_LINE_2 6
#define THROTTLE 20
#define STEERING 21
#define HALL_EFFECT_SENSOR 22
#define TOGGLE 23

//CH3 settings
#define MANUAL 0
#define AUTOMATIC 1
#define WP_MODE 2
#define AUX 3
#define RESET 4
#endif

#ifdef BLUETOOTH
#define SERIAL_OUT Serial2
#endif

#ifdef USB
#define SERIAL_OUT Serial
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


WILE E. COYOTE
1500 - stop
1550 - may not even move. DO NOT use
1560 - creeping speed
1575 - faster than creeping. probably a good starting speed


*/
