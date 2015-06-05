#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "../picomms.h"

#define SPEED 30.0

#define GAIN 0.1           
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.9

#define METER 1146
#define CENTIMETER 12.06
#define ROBOT_WIDTH 22.5
#define WHEEL_DIAMETER 9.5
#define UNIT CENTIMETER*60




double sqr(double x) {
	return x*x;
}

double hypotenuse(double x, double y) {
	return sqrt(sqr(x) + sqr(y));
}

void get_readings() {
	get_front_ir_dists(&frontLeftDist, &frontRightDist);
	get_side_ir_dists(&sideLeftDist, &sideRightDist);
	USDist=get_us_dist();
	printf("US: %d | ", USDist);
	printf("Left front dist: %d | ", frontLeftDist);
	printf("Left side dist: %d | Right side dist: %d\n", sideLeftDist, sideRightDist);
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
	current_angle = fmod(current_angle,360);
}

int new_speed(double target, double current) {
	double new_gain;
	double difference = fabs(target-current);
	double proportion = difference * GAIN;
	if ( NEW_GAIN_MIN <= proportion && proportion <= NEW_GAIN_MAX) {
		new_gain = proportion;
	}
	else 
		if (proportion > NEW_GAIN_MAX) {
			new_gain = NEW_GAIN_MAX;
		}
		else
		{
			new_gain = 1 - NEW_GAIN_MAX;
		}
	//printf("new_gain: %f\n",new_gain);
	return new_gain * SPEED;
}

void turn(int degrees) {
	double target_angle = orientation*90+degrees;  //modulus of negative numbers will make it turn more -- FIXED
	if (target_angle > 0)
		target_angle = fmod(orientation*90+degrees,360); 
	angle_change();
	if (target_angle == 360)             //workaround the modulus limitation on line 87 (fmod)
		target_angle = 359.8;			 //
	if (target_angle == -360)			 //
		target_angle = -359.8; 			 //
    if (degrees >= 0) {
    	if (current_angle > target_angle) {
			current_angle -= 360;
		}
	    while (current_angle < target_angle) {
	    	angle_change();
	    	//printf("Target angle: %f | Current angle: %f\n",target_angle,current_angle);
	        set_motors(new_speed(target_angle,current_angle),-new_speed(target_angle,current_angle));
	    }
	}
	else {
		if (current_angle < target_angle) {
			current_angle += 360;
		}
	    while (current_angle > target_angle) {
	        angle_change();
	        //printf("Target angle: %f | Current angle: %f\n",target_angle,current_angle);
	        set_motors(-new_speed(target_angle,current_angle),new_speed(target_angle,current_angle));
	    }
	}
    set_motors(0,0);
}

void drive_first_unit() {
    int initialleft,initialright,left = 0,right = 0;
    get_motor_encoders(&initialleft,&initialright);
    int speed = SPEED;
    while ((left+right)/2 <= (initialleft+initialright)/2+UNIT-18.75*CENTIMETER){
        get_motor_encoders(&left,&right);
        angle_change();
        if ((initialleft+initialright)/2+UNIT-18.75*CENTIMETER-(left+right)/2<=SPEED/4)
            speed = new_speed((initialleft+initialright)/2+UNIT-18.75*CENTIMETER,(left+right)/2);
        set_motors(speed,speed);
        log_trail();
    }
}

void drive(double distance) {
    int initialleft,initialright,left = 0,right = 0;
    get_motor_encoders(&initialleft,&initialright);
    int speed = SPEED;
    while ((left+right)/2 <= (initialleft+initialright)/2+distance){
        get_motor_encoders(&left,&right);
        angle_change();
        if ((initialleft+initialright)/2+distance-(left+right)/2 <= SPEED/4)
            speed = new_speed((initialleft+initialright)/2+distance,(left+right)/2);
        set_motors(speed,speed);
        log_trail();
    }
    set_motors(0,0);
}