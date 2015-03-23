// The turn function has to be improved.

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define DESIRED_FRONT_DIST 32
#define SPEED 30.0
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define ROBOT_WIDTH 22.5
#define CENTIMETER 12.06
#define UNIT CENTIMETER*60


int ticks_left = 0, ticks_right = 0, orientation = 0, position = 1, i;
int frontLeftDist, frontRightDist, sideLeftDist, sideRightDist, USDist;
double previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0, final_angle = M_PI / 2;
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
}

void turn(int angle) {
	if (angle==180)
		for (i = 0; i < 137; i++)
        	set_motors(50, -50);
    if (angle==90)
    	for (i = 0; i < 110; i++)
        	set_motors(30, -30);
    if (angle==-90)
    	for (i = 0; i < 68; i++) 
        	set_motors(-50, 50);
    set_motors(0, 0);
}

int new_speed() {
	double new_gain;
	double difference = DESIRED_FRONT_DIST - frontLeftDist;
	if (difference * GAIN <= NEW_GAIN_MAX && difference * GAIN >= NEW_GAIN_MIN)
	{
		new_gain = 1 - difference * GAIN;
	}
	else if (difference * GAIN > NEW_GAIN_MAX)
	{
		new_gain = 1 - NEW_GAIN_MAX;
	}
	else
	{
		new_gain = 1 - NEW_GAIN_MIN;
	}
	return new_gain * SPEED;
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
	delta_angle = (d_r - d_l) / ROBOT_WIDTH;
	total_d_x += d_m * cos(final_angle);
	total_d_y += d_m * sin(final_angle);
	final_angle += delta_angle;
}

void drive_first_unit() {
    int initialleft,initialright,left=0,right=0;
    get_motor_encoders(&initialleft,&initialright);
    int speed=SPEED;
    while ((left+right)/2<=(initialleft+initialright)/2+UNIT-18.75*CENTIMETER){
        get_motor_encoders(&left,&right);
        if ((initialleft+initialright)/2+UNIT-18.75*CENTIMETER-(left+right)/2<=SPEED/4)
            speed=new_speed();
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
        if ((initialleft+initialright)/2+UNIT-(left+right)/2<=SPEED/4)
            speed=new_speed();
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

void maze_explorer() {
	while (1){
		get_readings();
		mapper(orientation);
	    //angle_change();
	    setLinkedList(total_d_x, total_d_y);
	    if (sideLeftDist>24) {
	    	turn(-90); 
	    	orientation=(orientation-1)%4;
	    	printf("Turning -90\n");	
	    }
	    else if (USDist>30) {
			printf("Moving ahead\n");
		}
		else if (sideRightDist<24) {
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
	double error = fmod(final_angle - heading_r_to_p + 2*M_PI, 2*M_PI);
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
