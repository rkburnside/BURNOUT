//Header file variable
#define WAYPOINT_ACCEPT_RANGE 10 	//radius in # of feet in which to accept the waypoint
#define SERVO 2						//pin # for steering servo - green	
#define SERVO_STEERING_LIMIT_LEFT 130
#define SERVO_STEERING_LIMIT_RIGHT 50
#define THROTTLE 3					//pin # for throttle - yellow wire
#define SPEED_STOP 90
#define SPEED_SLOW 100
#define SPEED_MED 100
#define SPEED_FAST 110
#define SPEED_BREAKING 50
#define ERROR_GAIN 1.0				//1.0 full gain, .5 half as aggressive, .25 quarter aggressive

//Global Variables
double print_delay = 0;
double cross_track_error = 0;

//Gyro calibration
#define GYRO_SAMPLING_RATE 10 //sampling at 10ms
#define GYRO_CALIBRATION_NUMBER 3890318.0 //THIS NUMBER IS BASED ON SAMPLING TIME!!!
double gyro_sum=0, gyro_null=0, gyro_angle=0, angle_diff=0;		//for the gyro
double sample_time = 0;



//GPS Variables
float flat, flon, max_speed=0.0;
unsigned long age, date, time, chars;
unsigned short sentences, failed;
double waypoint_distance, waypoint_heading = 0.0;

//GPS Waypoints
int waypoint_num = 0;
const int waypoint_total = 8;	//<- should always be the same number of GPS waypoints
double gps_array[waypoint_total][2] = {
{39.53872443285608, -105.01673417986156},
{39.53864479537386, -105.01663091481448},
{39.53847310906244, -105.01641365588428},
{39.53839450554921, -105.01641365588428},
{39.53816386577954, -105.01665773690463},
{39.53840898515045, -105.0163989037347},
{39.53846690352515, -105.01639756263019},
{39.53872856986564, -105.0167408853841}
};

double unit_vectors[waypoint_total][2];