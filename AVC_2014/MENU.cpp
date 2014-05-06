//MENU FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES

//EXTERNAL VARIABLES
extern int mode;


//OBJECT DECLARATIONS


//PROGRAM FUNCTIONS
void menu_choices(){
	SERIAL_OUT.println();
	SERIAL_OUT.println("Main Menu");
	SERIAL_OUT.println("----------");
	SERIAL_OUT.println();
	SERIAL_OUT.println("WAYPOINT FUNCTIONS");
	SERIAL_OUT.println("----------");
	SERIAL_OUT.println("d = display waypoints");
	SERIAL_OUT.println("e = edit waypoint");
	SERIAL_OUT.println("i = import header waypoint values");
	SERIAL_OUT.println();
	SERIAL_OUT.println("GYRO FUNCTIONS");
	SERIAL_OUT.println("----------");
	SERIAL_OUT.println("a = watch angle");
	SERIAL_OUT.println("g = gyro calibration");
	SERIAL_OUT.println("w = watch gyro");
	SERIAL_OUT.println("k = gyro max turn rate");	
	SERIAL_OUT.println();
	SERIAL_OUT.println("MISCELLANEOUS FUNCTIONS");
	SERIAL_OUT.println("----------");
	SERIAL_OUT.println("c = click calibration");
	SERIAL_OUT.println("l = activate the FRICKIN LASER");
	SERIAL_OUT.println("m = combined mode and toggle state test");
	SERIAL_OUT.println("o = mode state test");
	SERIAL_OUT.println("t = switch state toggle test");
	SERIAL_OUT.println("s = steering calibration");
	SERIAL_OUT.println("v = servo test");
	SERIAL_OUT.println();
	SERIAL_OUT.println();
	SERIAL_OUT.println("x = exit. start setup routine for the race");
	SERIAL_OUT.println();
	return ;
}

void main_menu(){
	int loop = 1;
	get_mode();
	menu_choices();
	SERIAL_OUT.flush();
	while((loop) && (mode == MANUAL)){
		get_mode();
		if(SERIAL_OUT.available() > 0){
	 		switch(SERIAL_OUT.read()){
				case 'd':
					display_waypoints();
					menu_choices();
					break;
				case 'e':
					edit_waypoint();
					menu_choices();
					break;
				case 'i':
					import_waypoints();
					menu_choices();
					break;

				case 'a':
					watch_angle();
					menu_choices();
					break;
				case 'g':
					gyro_calibration();
					menu_choices();
					break;
				case 'w':
					watch_gyro();
					menu_choices();
					break;

				case 'k':
					//gyro_rate();
					lateral_accel();
					menu_choices();
					break;

				case 'c':
					click_calibration();
					menu_choices();
					break;
				case 'l':
					activate_the_frickin_laser();
					menu_choices();
					break;
				case 'm':
					mode_and_toggle_test();
					menu_choices();
					break;
				case 'o':
					mode_test();
					menu_choices();
					break;
				case 't':
					toggle_test();
					menu_choices();
					break;
				case 's':
					steering_calibration();
					menu_choices();
					break;
				case 'v':
					servo_test();
					menu_choices();
					break;

				case 'x':
					SERIAL_OUT.println();
					SERIAL_OUT.println();
					SERIAL_OUT.println("Setting up for the race");
					loop = 0;
					break;
				default:
					SERIAL_OUT.println("invalid entry. try again.");
					menu_choices();
					break;
			}
		delay(500);	// these lines are required to ensure that the menu is displayed after CH3 has been toggled. if removed, the get_mode() reacts too fast and causes the car to immediately proceed to the startup routine
		get_mode();
		}
	}
	return ;
}
