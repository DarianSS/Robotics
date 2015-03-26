// The turn function has to be improved.

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define DESIRED_FRONT_DIST 32
#define SPEED 30.0

#define GAIN 0.01           
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98

#define ROBOT_WIDTH 22.5
#define WHEEL_DIAMETER 9.5
#define TICKS_IN_METER 1146
#define CENTIMETER 12.06
#define UNIT CENTIMETER*60

#define METER 1146
#define SLOW_DIST METER*0.1


int ticks_left = 0, ticks_right = 0, orientation = 0, position = 1, i;
int frontLeftDist, frontRightDist, sideLeftDist, sideRightDist, USDist;
double previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0, current_angle = 0;
double cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;
double target_point_x = 0, target_point_y = 0;

typedef struct coords {
	double x;
	double y;
	struct coords *next, *previous;
} coords;

coords *start, *current, *new;

///////// Linked list initialisation //////////
coords *create(void) {
	coords *coordinateList;
	coordinateList = (coords *)malloc(sizeof(coords));
	coordinateList->x = 0.0;
	coordinateList->y = 0.0;
	if(coordinateList == NULL)
	{
		puts("Memory error");
		exit(1);
	}
	return(coordinateList);
}

void setLinkedList(double total_d_x, double total_d_y) {
	new = create();
	new->previous = current;
	current->next = new;
	current = current->next;
	current->x=total_d_x;
	current->y=total_d_y;
}
//////////////////////////////////////////

///////// Helper functions //////////
double sqr(double x){
	return x*x;
}

double hypotenuse(double x, double y){
	return sqrt(sqr(x) + sqr(y));
}

void get_readings() {
	get_front_ir_dists(&frontLeftDist, &frontRightDist);
	get_side_ir_dists(&sideLeftDist, &sideRightDist);
	USDist=get_us_dist();
	printf("US: %d\n", USDist);
}

void angle_change() {
	get_motor_encoders(&ticks_left, &ticks_right); 
	previous_cm_right = cm_right;
	previous_cm_left = cm_left;
	cm_left = ticks_left / CENTIMETER;
	cm_right = ticks_right / CENTIMETER;
	double d_l = cm_left - previous_cm_left;
	double d_r = cm_right - previous_cm_right;
	double d_m = (d_r + d_l) / 2.0;
	delta_angle = (d_l - d_r) / ROBOT_WIDTH * 180 / M_PI;
	total_d_x += d_m * cos(current_angle);
	total_d_y += d_m * sin(current_angle);
	current_angle += delta_angle;
	current_angle=fmod(current_angle,360);
}

int new_speed(double target, double current) {
	double new_gain;
	double difference = fabs(target-current);
	double proportion = difference * GAIN;
	if ( NEW_GAIN_MIN <= proportion && proportion <= NEW_GAIN_MAX) {
		new_gain = 1 - proportion;
	}
	else 
		if (proportion > NEW_GAIN_MAX) {
			new_gain = 1 - NEW_GAIN_MAX;
		}
		else
		{
			new_gain = 1 - NEW_GAIN_MIN;
		}
	return new_gain * SPEED;
}

void turn(int degrees){
	int target_angle = orientation*90+degrees;
    if (degrees>0)
	    while (current_angle < target_angle){
	    	angle_change();
	        set_motors(new_speed(target_angle,current_angle),-new_speed(target_angle,current_angle));
	        //set_motors(10,-10);
	    }
	else 
	    while (current_angle > target_angle){
	        angle_change();
	        set_motors(-new_speed(target_angle,current_angle),new_speed(target_angle,current_angle));
	        //set_motors(-10,10);
	    }
    set_motors(0,0);
}

/*   TURN FAILURES

void turn(int degrees){
    reset_motor_encoders();
    int initialleft,initialright,left=0,right=0;
    get_motor_encoders(&initialleft,&initialright);
    int speed=SPEED;
    if (degrees>0) 
	    while ((left-right)/2<=(initialleft+initialright)/2+abs(degrees)){
	        get_motor_encoders(&left,&right);
	        angle_change();
	        if ((initialleft+initialright)/2+degrees-(left-right)/2<=SLOW_DIST)
	            	speed=new_speed2((initialleft+initialright)/2+degrees-(left-right)/2);
	        set_motors(speed,-speed);
	    }
	else 
		while ((right-left)/2<=(initialleft+initialright)/2+abs(degrees)){
	        get_motor_encoders(&left,&right);
	        angle_change();
	        if ((initialleft+initialright)/2+degrees-(right-left)/2<=SLOW_DIST)   	
	            	speed=new_speed2((initialleft+initialright)/2+degrees-(right-left)/2);
	        set_motors(-speed,speed);
	    }
    set_motors(0,0);
}

// circumference = pi * diameter
// length of robot = 22.5 cm
// diameter of wheels = 9.5 cm
// 2 * pi radians in one circle
// 1146 ticks in a full circle
// circumference for wheel is 70.6858347 cm

double ticks_to_cm(int degrees_in_turn) {
	// Need to turn 90degrees
	int wheel_rounds = ((M_PI * ROBOT_WIDTH * degrees_in_turn) / 360) / WHEEL_DIAMETER; // each wheel goes this many rounds when making a 90 degree turn
	int rounds_to_ticks = wheel_rounds * TICKS_IN_METER / (M_PI * ROBOT_WIDTH) *2;
	return rounds_to_ticks;
}*/

void drive_first_unit() {
    int initialleft,initialright,left=0,right=0;
    get_motor_encoders(&initialleft,&initialright);
    int speed=SPEED;
    while ((left+right)/2<=(initialleft+initialright)/2+UNIT-18.75*CENTIMETER){
        get_motor_encoders(&left,&right);
        angle_change();
        if ((initialleft+initialright)/2+UNIT-18.75*CENTIMETER-(left+right)/2<=SPEED/4)
            speed=new_speed((initialleft+initialright)/2+UNIT-18.75*CENTIMETER,(left+right)/2);
        set_motors(speed,speed);
        log_trail();
    }
}

void drive_unit() {
    int initialleft,initialright,left=0,right=0;
    get_motor_encoders(&initialleft,&initialright);
    int speed=SPEED;
    while ((left+right)/2<=(initialleft+initialright)/2+UNIT){
        get_motor_encoders(&left,&right);
        angle_change();
        if ((initialleft+initialright)/2+UNIT-(left+right)/2<=SPEED/4)
            speed=new_speed((initialleft+initialright)/2+UNIT,(left+right)/2);
        set_motors(speed,speed);
        log_trail();
    }
    set_motors(0,0);
}
//////////////////////////////////////////

///////// Phase 1 //////////
void mapper(int orientation) {
	if (orientation==0) {
		position=position+4;               
	}
	if (orientation==1) {
		position=position+1;
	}
	if (orientation==2) {
		position=position-4;
	}
	if (orientation==3) {
		position=position-1;
	}	
}

/*void correction() {
	if (USDist>18 && 23>USDist) {
		if (sideLeftDist>18 && 22>sideLeftDist) {
			printf("Correction started, %f\n",180+orientation*90-current_angle);
			if (180+orientation*90-current_angle < -1) {
				while (current_angle > orientation*90) {
					printf("Case 1: %f | %i\n",current_angle,orientation*90);
					angle_change();
					set_motors(5,-5);
				}
			}
			else if (180+orientation*90-current_angle > 1) {
				while (current_angle < orientation*90) {
					printf("Case 2: %f | %i\n",current_angle,orientation*90);
					angle_change();
					set_motors(-5,5);
				}
			}
		}
		else {
			if (sideRightDist>18 && 22>sideRightDist) {
				printf("Correction started, %f\n",180+orientation*90-current_angle);
				if (180+orientation*90-current_angle < -1) {
					while (current_angle > orientation*90) {
						printf("Case 3: %f | %i\n",current_angle,orientation*90);
						angle_change();
						set_motors(-5,5);
					}
				}
				else if (180+orientation*90-current_angle > 1) {
					while (current_angle < orientation*90) {
						printf("Case 4: %f | %i\n",current_angle,orientation*90);
						angle_change();
						set_motors(5,-5);
					}
				}
			}
		}
	}
}*/

void maze_explorer() {
	while (1){
		get_readings();
		//correction();
		mapper(orientation);
	    //setLinkedList(total_d_x, total_d_y);

	    if (sideLeftDist>30) {
	    	turn(-90); 
	    	orientation=(orientation-1)%4;
	    	printf("Turning -90\n");	
	    }
	    else if (USDist>30) {
			printf("Moving ahead\n");
		}
		else if (sideRightDist<30) {
			turn(180);
			orientation=(orientation+2)%4;
			printf("Turning 180\n");
		}
		else {
			
			turn(90);
			orientation=(orientation+1)%4;
			printf("Turning 90\n");
		}
		drive_unit();
	}
}
//////////////////////////////////////////

///////// Trace functions //////////
void drive_to_target() {
	set_point(target_point_x, target_point_y);
	double heading_r_to_p = atan2(target_point_y - total_d_y, target_point_x - total_d_x);
	angle_change();
	double error = fmod(current_angle - heading_r_to_p + 2*M_PI, 2*M_PI);
	if(error > M_PI) {
		error -= 2 * M_PI;
	}
	int newtestspeedRight = 0;
	int newtestspeedLeft = 0;
	newtestspeedRight = (int)round(SPEED + (SPEED * error));
	newtestspeedLeft = (int)round(SPEED - (SPEED * error));
	set_motors(newtestspeedRight, newtestspeedLeft);
}


void trace() {
	while (current->previous != NULL) {
		if (hypotenuse(total_d_y - current ->y, total_d_x - current ->x) < 25.0) {
			current = current -> previous;
		}
		else {
			target_point_x = current -> x;
			target_point_y = current -> y;
			drive_to_target();
		}
	}
	while(hypotenuse(total_d_y - current->y, total_d_x - current->x) > 1.0){
		drive_to_target();
	}
}
//////////////////////////////////////////

int main() {
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(1, -45);

	start = create();
	start->previous = NULL;
	current = start;

	drive_first_unit();
	maze_explorer();
	set_motors(0, 0);
	return 0;
}
