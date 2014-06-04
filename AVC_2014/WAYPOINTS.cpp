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
	SERIAL_OUT.print("set WP #");
	SERIAL_OUT.print(wpw_count);
	SERIAL_OUT.print(": ");
	SERIAL_OUT.print(waypoint.x);
	SERIAL_OUT.print(" , ");
	SERIAL_OUT.println(waypoint.y);

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

	SERIAL_OUT.println();
	SERIAL_OUT.println("EEPROM clear");
	SERIAL_OUT.println();
	
	return ;
}

void import_waypoints(){
	eeprom_clear();
	
	wpw_count = 1;	//resets the counter to import correctly
	WAYPOINTS_STRING	//edit this in header file to change waypoints
	
	for(int i=0; i < WAYPOINT_COUNT; i++){
		waypoint.x = float(excel_waypoints[i][0]);
		waypoint.y = float(excel_waypoints[i][1]);
		EEPROM_writeAnything(wpw_count*WP_SIZE, waypoint);
		wpw_count++;
	}
	
	wpw_count = 1;	//resets the couter for autonomous mode
	display_waypoints();
	SERIAL_OUT.println("ALL POINTS IMPORTED");
	SERIAL_OUT.println();

	return ;
}

void display_waypoints(){
	SERIAL_OUT.println();
	
	for(int i=1; i <= WAYPOINT_COUNT; i++){
		EEPROM_readAnything(i*WP_SIZE, waypoint);

		Serial.print(i);
		Serial.print(": ");
		Serial.print(waypoint.x);
		Serial.print(" , ");
		Serial.println(waypoint.y);

		Serial2.print("#");
		Serial2.print(i);
		Serial2.print(",");
		Serial2.print(waypoint.x);
		Serial2.print(",");
		Serial2.println(waypoint.y);
	}

	SERIAL_OUT.println();

	return ;
}

void edit_waypoint(){
	while(1){
		display_waypoints();
		SERIAL_OUT.println();

		SERIAL_OUT.print("Edit wp #? ");
		int i = SERIAL_OUT.parseInt();
		EEPROM_readAnything(i*WP_SIZE, waypoint);
		
		SERIAL_OUT.println();
		SERIAL_OUT.print("current values: ");
		SERIAL_OUT.print(waypoint.x);
		SERIAL_OUT.print(" , ");
		SERIAL_OUT.println(waypoint.y);
		SERIAL_OUT.println();
		
		SERIAL_OUT.print("enter new coordinates \"x , y\": ");
		int x_temp = SERIAL_OUT.parseInt();
		int y_temp = SERIAL_OUT.parseInt();
		SERIAL_OUT.println();
		SERIAL_OUT.print("current values: ");
		SERIAL_OUT.print(waypoint.x);
		SERIAL_OUT.print(" , ");
		SERIAL_OUT.println(waypoint.y);
		SERIAL_OUT.print("new values: ");
		SERIAL_OUT.print(x_temp);
		SERIAL_OUT.print(" , ");
		SERIAL_OUT.println(y_temp);
		
		while(1){
			SERIAL_OUT.print("accept values (y=1, n=0)? ");
			int y_or_n = SERIAL_OUT.parseInt();
			if(y_or_n == 1){
				waypoint.x = float(x_temp);
				waypoint.y = float(y_temp);
				EEPROM_writeAnything(i*WP_SIZE, waypoint);
				SERIAL_OUT.println();
				SERIAL_OUT.println("waypoint changed");
				break;
			}
			else if(y_or_n == 0){
				SERIAL_OUT.println("no change made");
				break;
			}
			else SERIAL_OUT.println("invalid. try again");
		}

		SERIAL_OUT.println();
		SERIAL_OUT.print("edit another waypoint (y=1, n=0)? ");
		int n_or_y = SERIAL_OUT.parseInt();
		if(n_or_y == 1) ;
		else break;
	}
	
	SERIAL_OUT.println();
	
	return ;
}
