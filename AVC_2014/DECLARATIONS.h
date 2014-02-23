// functions declaration file. FUNCTIONS.cpp
#ifndef __FUNCTION_DECLARATIONS__
#define __FUNCTION_DECLARATIONS__

//gyro functions
void setup_mpu6050();
void read_FIFO();
void calculate_null();
void gyro_calibration();
void watch_angle();
void watch_gyro();


void get_mode();

#endif

#ifndef __INCLUDE_FILES__
#define __INCLUDE_FILES__

#include "AVC_2014.h"
#include "EEPROMAnything.h"
#include <I2Cdev.h> //declared 2x (1 here and 1 in the .ino)(shouldn't be, but arduino is dumb...see note below)
#include <MPU6050.h> //declared 2x (1 here and 1 in the .ino)(shouldn't be, but arduino is dumb...see note below)

//#include <MemoryFree.h>

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