#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "picomms.h"

#define SPEED 30.0

#define GAIN 0.1           
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.9

#define METER 1146
#define CENTIMETER 12.06
#define ROBOT_WIDTH 22.5
#define WHEEL_DIAMETER 9.5
#define UNIT CENTIMETER*60


int ticks_left = 0, ticks_right = 0, orientation = 0, position = 0, stop = 0, i;
int frontLeftDist, frontRightDist, sideLeftDist, sideRightDist, USDist;
double previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0, current_angle = 0;
double cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;

double sqr(double x) {
	return x*x;
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

void correction() {
	if (USDist < 50 && (sideRightDist < 30 || sideLeftDist < 30)) {
		if ((((USDist < 30) || (USDist > 32 && USDist < 50) ) && sideLeftDist < 30)       //US: 21-23 range //30-32
			|| ((sideLeftDist < 19) || (sideLeftDist > 21 && sideLeftDist < 30))) {	      //sideDists: 19-21 range
				printf("Correction case 1:\n");
				int delta_x = sideLeftDist+11-30;
				int delta_y = USDist-1-30;  
				double alpha = -atan2(delta_x, delta_y) * 180 / M_PI;
				printf("alpha: %f || delta_x: %d | delta_y: %d || drive_dist: %f\n", alpha, delta_x, delta_y, sqrt(sqr(delta_x)+sqr(delta_y)));
				turn(alpha);
				//usleep(1000000);
				drive(sqrt(sqr(delta_x)+sqr(delta_y)) * CENTIMETER);
				turn(0);
				//usleep(1000000);

				printf("Post-correction: ");
				get_readings();
			}
			else if ((((USDist < 30) || (USDist > 32 && USDist < 50)) && sideRightDist < 30)
				|| ((sideRightDist < 19) || (sideRightDist > 21 && sideRightDist < 30))) {
					printf("Correction case 2:\n");
					int delta_x = 30-(sideRightDist+10);
					int delta_y = USDist+8-30;
					double alpha = -atan2(delta_x,delta_y) * 180 / M_PI;
					printf("alpha: %f || delta_x: %d | delta_y: %d || drive_dist: %f\n", alpha, delta_x, delta_y, sqrt(sqr(delta_x)+sqr(delta_y)));
					turn(alpha);
					drive(sqrt(sqr(delta_x)+sqr(delta_y)) * CENTIMETER);
					turn(0);

					printf("Post-correction: ");
					get_readings();
			}			
	}
	else if (USDist >= 50 && sideLeftDist < 30) {
		int dif = sideLeftDist*100/frontLeftDist; printf("%d\n",dif);
		if (dif < 68) {
			printf("Correction case 3:\n");
			while (dif < 75) {
				printf("3: ");
				get_readings();
				dif = sideLeftDist*100/frontLeftDist; printf("%d\n",dif);
				set_motors(-5, 5);
			}
			set_motors(0,0);
		}
		else if (dif > 78) {
			printf("Correction case 4:\n");
			while (dif > 75) {
				printf("4: ");
				get_readings();
				dif = sideLeftDist*100/frontLeftDist; printf("%d\n",dif);
				set_motors(5,-5);
			}
			set_motors(0,0);	
		}
	}
}

int main() {
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(LEFT, -45);
	set_ir_angle(RIGHT, 45);
	usleep(1000000);

	get_readings();
	correction();
	//drive_first_unit();
	//turn(180);
	//drive_first_unit();
	get_readings();
	return 0;
}