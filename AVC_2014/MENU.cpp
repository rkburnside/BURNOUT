//MENU FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES

//EXTERNAL VARIABLES
extern int mode;


//OBJECT DECLARATIONS


//PROGRAM FUNCTIONS
void menu_choices(){
	Serial2.println();
	Serial2.println("Main Menu");
	Serial2.println("----------");
	Serial2.println();
	Serial2.println("WAYPOINT FUNCTIONS");
	Serial2.println("----------");
	Serial2.println("d = display waypoints");
	Serial2.println("e = edit waypoint");
	Serial2.println("i = import header waypoint values");
	Serial2.println();
	Serial2.println("GYRO FUNCTIONS");
	Serial2.println("----------");
	Serial2.println("a = watch angle");
	Serial2.println("g = gyro calibration");
	Serial2.println("w = watch gyro");
	Serial2.println();
	Serial2.println("MISCELLANEOUS FUNCTIONS");
	Serial2.println("----------");
	Serial2.println("c = click calibration");
	Serial2.println("l = activate the FRICKIN LASER");
	Serial2.println("m = combined mode and toggle state test");
	Serial2.println("o = mode state test");
	Serial2.println("t = switch state toggle test");
	Serial2.println("s = steering calibration");
	Serial2.println("v = servo test");
	Serial2.println();
	Serial2.println();
	Serial2.println("x = exit. start setup routine for the race");
	Serial2.println();
	return ;
}

void main_menu(){
	int loop = 1;
	get_mode();
	menu_choices();
	Serial2.flush();
	while((loop) && (mode == MANUAL)){
		get_mode();
		if(Serial2.available() > 0){
	 		switch(Serial2.read()){
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
					Serial2.println();
					Serial2.println();
					Serial2.println("Setting up for the race");
					loop = 0;
					break;
				default:
					Serial2.println("invalid entry. try again.");
					menu_choices();
					break;
			}
		delay(500);
		get_mode();
		}
	}
	return ;
}
