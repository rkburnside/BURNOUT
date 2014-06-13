//AVC SETTINGS
#define RR			//use either MM (minuteman) or RR (roadrunner) or WC (Wile E. Coyote)
#define USB 	//use either BLUETOOTH or USB to define the serial port for program output

#define WAYPOINT_COUNT 19
#define WAYPOINTS_STRING \
int excel_waypoints[19][2] = {{208,767}, {143,2463}, {-972,2905}, {-2015,2518}, {-4976,2496}, {-5236,1953}, {-5241,1058}, {-5245,715}, {-5245,-1650}, {-4775,-1650}, {-65,-1800}, {-55,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}};

#ifdef MM
//WAYPOINT AND SPEED PARAMETERS
#define WAYPOINT_ACCEPT 50	//waypoint acceptance radius in inches
#define S_STOP 1500				// some default values:
#define S_LOW 1540				//S1 1500, S2 1540, S3 1560, S4 1600, S5 1650, SB 1300
							//S1 1500, S2 1560, S3 1580, S4 2000, SB 1250
#define S_HIGH 1650				//This is the speed for negotiating wp's 
#define L1 10
#define L2 20
#define L3 350
#define L4 200
#define XGYROOFFSET 88	//85
#define YGYROOFFSET -72	//-70
#define ZGYROOFFSET -24	//-22
#define PATH_FOLLOWING 1
#define LOOK_AHEAD 120
#define SPEED_TOGGLE_ANGLE 20.0


//SENSOR PARAMETERS
//#define GYRO_CAL 470868410	//this has to be measured by rotating the gyro 360 deg. and reading the output
#define GYRO_CAL 233300000		//this has to be measured by rotating the gyro 360 deg. and reading the output
#define STEER_ADJUST 1425		//steering adjustment factor
#define STEER_GAIN 3500			//proportional gain, default it 4.0
#define CLICK_INCHES 4.66		//conversion factor, inches per click

//FIXED PARAMETERS
#define CAR_NAME "***MINUTEMAN***" //car name
#define CLICK_MAX 1			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3
#define WP_SIZE 20 			//number of bytes for each waypoint

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

#ifdef RR
//WAYPOINT PARAMETERS
#define WAYPOINT_ACCEPT 120	//waypoint acceptance radius: speed = 11650 -> 48, speed = 1850 -> 180

//SPEED PARAMETERS
#define S_STOP 1500				//stationary speed
#define S_LOW 1650				//slow speed: 1600 ~ 1650
#define S_HIGH 1750				//top speed: 1650 ~ 1800

//STEERING PARAMETERS
#define L1 5000
#define L2 20000
#define L3 200
#define L4 350
#define STEER_ADJUST 1505			//steering adjustment factor
#define STEER_GAIN 2000				//proportional gain, 4000 = servo slammed, 2000 = servo slams less, 300 = supposedly OK
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
#define CLICK_MAX 1			//in the main loop, watch clicks and wait for it to reach CLICK_MAX, then calculate position, default 3

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

#ifdef WC
//WAYPOINT PARAMETERS
#define WAYPOINT_ACCEPT 48		//waypoint acceptance radius: speed = 11650 -> 48, speed = 1850 -> 180

//SPEED PARAMETERS
#define S_STOP 1500				//stationary speed
#define S_LOW 1600				//slow speed: 1600 ~ 1625
#define S_HIGH 1650				//top speed: 1650 ~ 1800

//STEERING PARAMETERS
#define L1 5000
#define L2 20000
#define L3 100
#define L4 250
#define STEER_ADJUST 1465			//steering adjustment factor
//***Note that Wile E. Coyote servo is opposite of roadrunner. As a result, the steering gain AND the gyro cal numbers are "-"
#define STEER_GAIN -2000			//proportional gain, 4000 = servo slammed, 2000 = servo slams less, 300 = supposedly OK
#define PATH_FOLLOWING 1
#define LOOK_AHEAD 240				//120 = too high, 20 = too low, 50 = aggressive, but good, 80 = not aggressive, but OK
#define SPEED_TOGGLE_ANGLE 20.0		//20 +/- 5 degrees is good

//GYRO PARAMETERS
#define GYRO_CAL 234401167			//this has to be measured by rotating the gyro 360 deg. and reading the output and then dividing by the number of rotations and by 2 to get a 180-deg number
#define XGYROOFFSET -30
#define YGYROOFFSET 32
#define ZGYROOFFSET -10

//MISCELLANEOUS PARAMETERS
#define CAR_NAME "***WILE E. COYOTE***" //car name
#define WP_SIZE 20 				//number of bytes for each waypoint
#define CLICK_INCHES 8.6330936	//69~70 clicks in 25 feet (300 inches)
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