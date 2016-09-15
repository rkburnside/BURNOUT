//FUNCTIONS DECLARATIONS

#ifndef __FUNCTION_DECLARATIONS__
#define __FUNCTION_DECLARATIONS__

//GYRO FUNCTIONS
void setup_mpu6050();
void read_FIFO();
void clear_i2c();
void calculate_null();
void gyro_calibration();
void watch_angle();
void watch_gyro();
void reset_FIFO();
void gyro_rate();
//void lateral_accel();
int get_gyro_rate();
int get_accel_rate();

//WAYPOINT FUNCTIONS
void set_waypoint();
void read_waypoint();
void eeprom_clear();
void import_waypoints();
void display_waypoints();
void edit_waypoint();
void reset_waypoints();

//NAVIGATION FUNCTIONS
void end_run();
void update_cross_product();
void update_steering();
void calculate_speed();
void speed();
void cal_wp_accept();
void update_position();
void update_waypoint();
void calculate_look_ahead();
void print_telemetry(); //compare to print coordinates, data, parameters
void print_coordinates(); //compare to print telemetry
void print_data(); //compare to print telemetry
void print_parameters(); //compare to print telemetry
void cal_steer_lim();

//CALIBRATION FUNCTIONS
void steering_calibration();
void click_calibration();
void click_calibration_increment();
void servo_test();
void mode_test();
void toggle_test();
void mode_and_toggle_test();

//STATE STATUS FUNCTIONS
void get_mode();


//MENU
void main_menu();

#endif

#ifndef __INCLUDE_FILES__
#define __INCLUDE_FILES__

#include "AVC_2016.h"
#include "EEPROMAnything.h"
#include <Servo.h>
#include <I2Cdev.h> //declared 2x (1 here and 1 in the .ino)(shouldn't be, but arduino is dumb...see note below)
#include <MPU6050.h> //declared 2x (1 here and 1 in the .ino)(shouldn't be, but arduino is dumb...see note below)

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

//#include "MPU6050_6Axis_MotionApps20.h"

//#include "new_gyro.h"

/* arduino ide is stupid...
see: http://jamesreubenknowles.com/including-libraries-in-the-arduino-ide-1625
Any Arduino libraries that are needed by the files outside of the sketch (.ino) file must also be listed in the sketch file itself.
The sketch is parsed for include files. The sketch, all included header files, and the corresponding source files, are copied to another directory for compiling. From that directory, library-based include files are NOT available unless they are included in the sketch and copied to the build directory.
*/

#endif

#ifndef __STRUCTURES_DEFINITION__
#define __STRUCTURES_DEFINITION__

struct position_structure {

/* Using structures to contain location information. Will record old position
and new position. The actual structures will be position[0] and position[1],
but will use pointers old_pos and new_pos to access them. This way we can simply
swap the pointers instead of copying entire structure from one to the other. Access
data in the structures as follows: old_pos->x or new_pos->time, etc. this is equivalent
to (*old_pos).x.*/

	double x;
	double y;
	//boolean last;
};

#endif
