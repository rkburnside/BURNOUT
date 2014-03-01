//WAYPOINT FUNCTIONS

//#INCLUDE FILES
#include "DECLARATIONS.h"


//INTERNAL VARIABLES
byte wpr_count=1;
static byte wpw_count=1;


//EXTERNAL VARIABLES
extern bool aux;
extern double x, y;


//OBJECT DECLARATIONS
position_structure waypoint;


//PROGRAM FUNCTIONS
void set_waypoint(){ //CLEAR
	waypoint.x = x;
	waypoint.y = y;
	//waypoint.last = false
	EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
	Serial.print("set WP #");
	Serial.print(wpw_count);
	Serial.print(": ");
	Serial.print(waypoint.x*CLICK_INCHES);
	Serial.print(" , ");
	Serial.println(waypoint.y*CLICK_INCHES);
	wpw_count++;
	while(aux) get_mode();

	return ;
}    

void read_waypoint(){ //CLEAR
	EEPROM_readAnything(wpr_count*WP_SIZE, waypoint);
	
	return ;
}

void eeprom_clear(){  // CLEAR  //EEPROM Clear
	// write a 0 to all 1024 bytes of the EEPROM
	for(int i = 0; i < 1024; i++) EEPROM.write(i, 0);

	Serial.println();
	Serial.println("EEPROM clear");
	Serial.println();
	
	return ;
}

void import_waypoints(){
	eeprom_clear();
	
	wpw_count = 1;	//resets the counter to import correctly
	WAYPOINTS_STRING    //edit this in header file to change waypoints
	
	for(int i=0; i < WAYPOINT_COUNT; i++){
		waypoint.x = float(excel_waypoints[i][0])/CLICK_INCHES;
		waypoint.y = float(excel_waypoints[i][1])/CLICK_INCHES;
		EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
		wpw_count++;
	}
	
	wpw_count = 1;	//resets the couter for autonomous mode
	display_waypoints();
	Serial.println("ALL POINTS IMPORTED");
	Serial.println();

	return ;
}

void display_waypoints(){
	Serial.println();
	for(int i=1; i <= WAYPOINT_COUNT; i++){
		EEPROM_readAnything(i*WP_SIZE, waypoint);
		Serial.print(i);
		Serial.print(": ");
		Serial.print(waypoint.x*CLICK_INCHES);
		Serial.print(" , ");
		Serial.println(waypoint.y*CLICK_INCHES);
	}
	Serial.println();

	return ;
}

void edit_waypoint(){
	while(1){
		display_waypoints();
		Serial.println();

		Serial.print("Edit wp #? ");
		int i = Serial.parseInt();
		EEPROM_readAnything(i*WP_SIZE, waypoint);
		
		Serial.println();
		Serial.print("current values: ");
		Serial.print(waypoint.x*CLICK_INCHES);
		Serial.print(" , ");
		Serial.println(waypoint.y*CLICK_INCHES);
		Serial.println();
		
		Serial.print("enter new coordinates \"x , y\": ");
		int x_temp = Serial.parseInt();
		int y_temp = Serial.parseInt();
		Serial.println();
		Serial.print("current values: ");
		Serial.print(waypoint.x*CLICK_INCHES);
		Serial.print(" , ");
		Serial.println(waypoint.y*CLICK_INCHES);
		Serial.print("new values: ");
		Serial.print(x_temp);
		Serial.print(" , ");
		Serial.println(y_temp);
		
		while(1){
			Serial.print("accept values (y=1, n=0)? ");
			int y_or_n = Serial.parseInt();
			if(y_or_n == 1){
				waypoint.x = float(x_temp)/CLICK_INCHES;
				waypoint.y = float(y_temp)/CLICK_INCHES;
				EEPROM_writeAnything(i*WP_SIZE, waypoint);
				Serial.println();
				Serial.println("waypoint changed");
				break;
			}
			else if(y_or_n == 0){
				Serial.println("no change made");
				break;
			}
			else Serial.println("invalid. try again");
		}

		Serial.println();
		Serial.print("edit another waypoint (y=1, n=0)? ");
		int n_or_y = Serial.parseInt();
		if(n_or_y == 1) ;
		else break;
	}
	
	Serial.println();
	
	return ;
}
