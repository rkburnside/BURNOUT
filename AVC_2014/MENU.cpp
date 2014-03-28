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
	Serial2.println("a = watch angle");
	Serial2.println("d = display waypoints");
	Serial2.println("e = edit waypoint");
	Serial2.println("f = click calibration");
	Serial2.println("i = import header waypoint values");
	Serial2.println("l = gyro calibration");
	Serial2.println("m = free memory");
	Serial2.println("s = steering calibration");
	Serial2.println("v = servo test");
	Serial2.println("w = watch gyro");
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
				case 'a':
					watch_angle();
					menu_choices();
					break;
				case 'd':
					display_waypoints();
					menu_choices();
					break;
				case 'e':
					edit_waypoint();
					menu_choices();
					break;
				case 'f':
					click_calibration();
					menu_choices();
					break;
				case 'i':
					import_waypoints();
					menu_choices();
					break;
				case 'l':
					gyro_calibration();
					menu_choices();
					break;
				case 'm':
					Serial2.println();
					Serial2.println();
					Serial2.print("available memory: ");
					//Serial2.println(freeMemory());
					Serial2.println();
					Serial2.println();
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
				case 'w':
					watch_gyro();
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
