//MENU FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES


//EXTERNAL VARIABLES
extern int mode;


//OBJECT DECLARATIONS


//PROGRAM FUNCTIONS
void menu_choices(){
	Serial.println();
	Serial.println("Main Menu");
	Serial.println("----------");
	Serial.println("a = watch angle");
	Serial.println("d = display waypoints");
	Serial.println("e = edit waypoint");
	Serial.println("f = click calibration");
	Serial.println("i = import header waypoint values");
	Serial.println("l = gyro calibration");
	Serial.println("m = free memory");
	Serial.println("s = steering calibration");
	Serial.println("w = watch gyro");
	Serial.println("x = exit. start setup routine for the race");
	Serial.println();
	return ;
}

void main_menu(){
	int loop = 1;
	get_mode();
	menu_choices();
	Serial.flush();
	while((loop) && (mode == MANUAL)){
		get_mode();
		if(Serial.available() > 0){
	 		switch(Serial.read()){
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
					Serial.println();
					Serial.println();
					Serial.print("available memory: ");
					//Serial.println(freeMemory());
					Serial.println();
					Serial.println();
					menu_choices();
					break;
				case 's':
					steering_calibration();
					menu_choices();
					break;
				case 'w':
					watch_gyro();
					menu_choices();
					break;
				case 'x':
					Serial.println();
					Serial.println();
					Serial.println("Setting up for the race");
					loop = 0;
					break;
				default:
					Serial.println("invalid entry. try again.");
					menu_choices();
					break;
			}
		delay(500);
		get_mode();
		}
	}
	return ;
}
