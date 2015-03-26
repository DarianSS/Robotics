// Midterm Task Spring 2015
// Code Created by Hekla Helgadottir and Darian Sastre

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define DESIRED_FRONT_DIST 32
#define SPEED 70.0
#define GAIN 0.1
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.98
#define ROBOT_WIDTH 22.5
#define CENTIMETER 12.06

int ticks_left = 0, ticks_right = 0;
int frontLeftDist, frontRightDist, i;
double previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0, final_angle = M_PI / 2;
double cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;
double target_point_x = 0, target_point_y = 0;

void setLinkedList(double total_d_x, double total_d_y);

double sqr(double x){
	return x*x;
}
double hypotenuse(double x, double y){
	return sqrt(sqr(x) + sqr(y));
}

typedef struct coords {
	double x;
	double y;
	struct coords *next, *previous;
} coords;

coords *start, *current, *new;

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

void readings() {
	get_front_ir_dists(&frontLeftDist, &frontRightDist);
}

int new_speed() {
	double new_gain;
	double difference = DESIRED_FRONT_DIST - frontLeftDist;
	if (difference * GAIN <= NEW_GAIN_MAX && difference * GAIN >= NEW_GAIN_MIN) {
		new_gain = 1 - difference * GAIN;
	}
	else 
		if (difference * GAIN > NEW_GAIN_MAX) {
			new_gain = 1 - NEW_GAIN_MAX;
		}
		else
		{
			new_gain = 1 - NEW_GAIN_MIN;
		}
	return new_gain * SPEED;
}

void setLinkedList(double total_d_x, double total_d_y) {
	new = create();
	new->previous = current;
	current->next = new;
	current = current->next;
	current->x=total_d_x;
	current->y=total_d_y;
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

void move() {
	while (1){
		readings();
	    angle_change();
	    printf("%f | %f\n",total_d_x,total_d_y);
	    setLinkedList(total_d_x, total_d_y);
		if (frontRightDist < 28 && frontRightDist >= 19) {
			set_motors(5,5);
		}
		else if (frontRightDist < 19) {
			break;
		}
		else if (frontLeftDist <= DESIRED_FRONT_DIST+1 && frontLeftDist >= DESIRED_FRONT_DIST-1) {
			set_motors(SPEED, SPEED);
		}
		else if (frontLeftDist > DESIRED_FRONT_DIST) {
			set_motors(new_speed(), SPEED);
		}
		else {
			set_motors(SPEED, new_speed());
		}
	}
}

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

void turn() {
    for (i = 0; i < 137; i++) {
        set_motors(50, -50);
    }
    set_motors(0, 0);
}

int main() {
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(1, -45);
	start = create();
	start->previous = NULL;
	current = start;
	move();
	turn();
	trace();
	set_motors(0, 0);
	return 0;
}
