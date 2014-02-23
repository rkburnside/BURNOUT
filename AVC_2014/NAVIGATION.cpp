//AVC_2014 multi-file branch, navigation code

//#INCLUDE FILES
#include "DECLARATIONS.h"



//INTERNAL VARIABLES


//EXTERNAL VARIABLES
extern int steer_us;


//OBJECT DECLARATIONS
extern Servo steering, esc;


//PROGRAM FUNCTIONS



void end_run() {    // go straight forward, slowly at last waypoint
	esc.writeMicroseconds(S2); //reduce speed
	steer_us = STEER_ADJUST;  // go straight
	while(true); //loop endlessly
}













