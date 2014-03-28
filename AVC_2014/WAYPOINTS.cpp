//WAYPOINT FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
byte wpr_count=1;
static byte wpw_count=1;


//EXTERNAL VARIABLES
extern int mode;
extern double x, y;


//OBJECT DECLARATIONS
position_structure waypoint;


//PROGRAM FUNCTIONS
void set_waypoint(){ //CLEAR
	waypoint.x = x;
	waypoint.y = y;
	//waypoint.last = false
	EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
	Serial2.print("set WP #");
	Serial2.print(wpw_count);
	Serial2.print(": ");
	Serial2.print(waypoint.x*CLICK_INCHES);
	Serial2.print(" , ");
	Serial2.println(waypoint.y*CLICK_INCHES);

	wpw_count++;
	while(mode == WP_MODE) get_mode();

	return ;
}

void read_waypoint(){ //CLEAR
	EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
	
	return ;
}

void eeprom_clear(){  // CLEAR  //EEPROM Clear
	// write a 0 to all 1024 bytes of the EEPROM
	for(int i = 0; i < 1024; i++) EEPROM.write(i, 0);

	Serial2.println();
	Serial2.println("EEPROM clear");
	Serial2.println();
	
	return ;
}

void import_waypoints(){
	eeprom_clear();
	
	wpw_count = 1;	//resets the counter to import correctly
	WAYPOINTS_STRING	//edit this in header file to change waypoints
	
	for(int i=0; i < WAYPOINT_COUNT; i++){
		waypoint.x = float(excel_waypoints[i][0])/CLICK_INCHES;
		waypoint.y = float(excel_waypoints[i][1])/CLICK_INCHES;
		EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
		wpw_count++;
	}
	
	wpw_count = 1;	//resets the couter for autonomous mode
	display_waypoints();
	Serial2.println("ALL POINTS IMPORTED");
	Serial2.println();

	return ;
}

void display_waypoints(){
	Serial.begin(115200);

	Serial2.println();
	for(int i=1; i <= WAYPOINT_COUNT; i++){
		EEPROM_readAnything(i*WP_SIZE, waypoint);

		Serial.print(i);
		Serial.print(": ");
		Serial.print(waypoint.x*CLICK_INCHES);
		Serial.print(" , ");
		Serial.println(waypoint.y*CLICK_INCHES);

		Serial2.print(i);
		Serial2.print(": ");
		Serial2.print(waypoint.x*CLICK_INCHES);
		Serial2.print(" , ");
		Serial2.println(waypoint.y*CLICK_INCHES);
	}

	Serial.println();
	Serial.println();
	Serial.end();

	Serial2.println();

	return ;
}

void edit_waypoint(){
	while(1){
		display_waypoints();
		Serial2.println();

		Serial2.print("Edit wp #? ");
		int i = Serial2.parseInt();
		EEPROM_readAnything(i*WP_SIZE, waypoint);
		
		Serial2.println();
		Serial2.print("current values: ");
		Serial2.print(waypoint.x*CLICK_INCHES);
		Serial2.print(" , ");
		Serial2.println(waypoint.y*CLICK_INCHES);
		Serial2.println();
		
		Serial2.print("enter new coordinates \"x , y\": ");
		int x_temp = Serial2.parseInt();
		int y_temp = Serial2.parseInt();
		Serial2.println();
		Serial2.print("current values: ");
		Serial2.print(waypoint.x*CLICK_INCHES);
		Serial2.print(" , ");
		Serial2.println(waypoint.y*CLICK_INCHES);
		Serial2.print("new values: ");
		Serial2.print(x_temp);
		Serial2.print(" , ");
		Serial2.println(y_temp);
		
		while(1){
			Serial2.print("accept values (y=1, n=0)? ");
			int y_or_n = Serial2.parseInt();
			if(y_or_n == 1){
				waypoint.x = float(x_temp)/CLICK_INCHES;
				waypoint.y = float(y_temp)/CLICK_INCHES;
				EEPROM_writeAnything(i*WP_SIZE, waypoint);
				Serial2.println();
				Serial2.println("waypoint changed");
				break;
			}
			else if(y_or_n == 0){
				Serial2.println("no change made");
				break;
			}
			else Serial2.println("invalid. try again");
		}

		Serial2.println();
		Serial2.print("edit another waypoint (y=1, n=0)? ");
		int n_or_y = Serial2.parseInt();
		if(n_or_y == 1) ;
		else break;
	}
	
	Serial2.println();
	
	return ;
}
