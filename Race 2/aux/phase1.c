#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "../picomms.h"
#include "helpers.c"

#define SPEED 30.0

#define GAIN 0.1           
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.9

#define METER 1146
#define CENTIMETER 12.06
#define ROBOT_WIDTH 22.5
#define WHEEL_DIAMETER 9.5
#define UNIT CENTIMETER*60




void mapper(int orientation) {   //only updates position at this time
	if (orientation == 0) {
		if (position == 0)
			position = position+1;
		else
			position = position+4;               
	}
	if (orientation == 1) {
		position = position+1;
	}
	if (orientation == 2) {
		if (position == 1)
			position = position-1;
		else
			position = position-4;
	}
	if (orientation == 3) {
		position = position-1;
	}	
}

void stopper() {
	if (position == 0 && stop != 0) {
		usleep(1000000);
	}
	if (position == 16) {
		if (stop == 0)
			stop++;
		else
			exit(0);
	}
}

void correction() {
	if (USDist < 40 && (sideRightDist < 30 || sideLeftDist < 30)) {
		if ((((USDist < 21) || (USDist > 23 && USDist < 40) ) && sideLeftDist < 30)       //US: 21-23 range
			|| ((sideLeftDist < 19) || (sideLeftDist > 21 && sideLeftDist < 30))) {	      //sideDists: 19-21 range
				printf("Correction case 1:\n");
				int delta_x = sideLeftDist+10-30;
				int delta_y = USDist+8-30;
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
			else if ((((USDist < 21) || (USDist > 23 && USDist < 40)) && sideRightDist < 30)
				|| ((sideRightDist < 19) || (sideRightDist > 21 && sideRightDist < 30))) {
					printf("Correction case 2:\n");
					int delta_x = 30-(sideRightDist+10);
					int delta_y = USDist+8-30;
					double alpha = -atan2(delta_x,delta_y) * 180 / M_PI;
					printf("alpha: %f || delta_x: %d | delta_y: %d\n", alpha, delta_x, delta_y);
					turn(alpha);
					drive(sqrt(sqr(delta_x)+sqr(delta_y)) * CENTIMETER);
					turn(0);

					printf("Post-correction: ");
					get_readings();
			}			
	}
	else if (USDist >= 40 && sideLeftDist < 30) {
		if (sideLeftDist+1 < frontLeftDist) {
			printf("Correction case 3:\n");
			while (sideLeftDist+1 < frontLeftDist) {
				printf("3: ");
				get_readings();
				set_motors(-5,5);
			}
		}
		else if (sideLeftDist+1 > frontLeftDist) {
			printf("Correction case 4:\n");
			while (sideLeftDist > frontLeftDist) {
				printf("4: ");
				get_readings();
				set_motors(5,-5);
			}
		}
	}
}

void maze_explorer() {
	while (1) {
		get_readings();
		correction();
		mapper(orientation);
		stopper();
	    //setLinkedList(total_d_x, total_d_y);

	    if (sideLeftDist > 30) {
	    	printf("Turning -90\n");
	    	turn(-90); 
	    	printf("Finished Turning -90\n");
	    	orientation = (orientation+3)%4;    
	    }
	    else if (USDist < 40) {
	    	if (sideRightDist < 30) {
				printf("Turning 180\n");
				turn(180);
				printf("Finished Turning 180\n");
				orientation = (orientation+2)%4;
			}
			else {
				printf("Turning 90\n");
				turn(90);
				printf("Finished Turning 90\n");
				orientation = (orientation+1)%4;
			}
		}
		printf("Moving ahead\n");
		drive(UNIT-10);								//The -10 is for precision
		printf("Finished Moving ahead\n");
	}
}