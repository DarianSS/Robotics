#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "picomms.h"
//#include "aux/helpers.c"
//#include "aux/phase1.c"

#define INF 1<<16
#define SPEED 30.0

#define GAIN 0.1           
#define NEW_GAIN_MIN 0.3
#define NEW_GAIN_MAX 0.9

#define METER 1146
#define CENTIMETER 12.06
#define ROBOT_WIDTH 22.5
#define WHEEL_DIAMETER 9.5
#define UNIT CENTIMETER*60


int ticks_left = 0, ticks_right = 0, orientation = 0, position = 0, stop = 0, maze[24][24], paths[17][17];
int frontLeftDist, frontRightDist, sideLeftDist, sideRightDist, USDist;
double previous_cm_right = 0, previous_cm_left = 0, delta_angle = 0, current_angle = 0, final_angle = M_PI / 2;;
double cm_right = 0, cm_left = 0, total_d_y = 0, total_d_x = 0;
double target_point_x = 0, target_point_y = 0;

typedef struct coords {
	double x, y;
	struct coords *next;
} coords;

typedef struct path {
	int square;
	struct path *next;
} path;

coords *start, *current, *new;
path *p_start, *p_current;

///////// Linked list initialisation //////////
coords *create(void) {
	coords *new;
	new = (coords *)malloc(sizeof(coords));
	new -> x = 0.0;
	new -> y = 0.0;
	if (new == NULL)
	{
		puts("Memory error");
		exit(1);
	}
	return(new);
}

path *p_create(void) {
	path *new;
	new = (path *)malloc(sizeof(path));
	new -> square = 0;
	if (new == NULL)
	{
		puts("Memory error");
		exit(1);
	}
	return(new);
}

void setLinkedList(double total_d_x, double total_d_y) {
	new = create();
	current -> next = new;
	current = current -> next;
	current -> x = total_d_x;
	current -> y = total_d_y;
}
//////////////////////////////////////////

///////// Helper functions //////////
void initialisation() {
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(LEFT, -45);

	for (int i=0; i<=16; ++i) 
		for (int j=0; j<=16; ++j)
			maze[i][j] = INF;

	maze[0][1] = 1;
	maze[1][0] = 1;

	current = create();
	start = current;

	p_current = p_create();
	p_start = p_current;
}

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
	//printf("US: %d | ", USDist);
	//printf("Left front dist: %d | ", frontLeftDist);
	//printf("Left side dist: %d | Right side dist: %d\n", sideLeftDist, sideRightDist);
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

void angle_change_tracing() {
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
//////////////////////////////////////////

///////// Phase 1 //////////
void mapper(int orientation) {   
	if (orientation == 0) {
		if (position == 0)
			position = position+1;
		else {
			position = position+4;
		}   
		if (USDist > 50) {
			maze[position][position+4] = maze[position+4][position] = 1;
		}
		if (sideRightDist > 30) {
			maze[position][position+1] = maze[position+1][position] = 1;
		} 
		if (sideLeftDist > 30) {
			maze[position][position-1] = maze[position-1][position] = 1;
		}         
	}
	if (orientation == 1) {
		position = position+1;
		if (USDist > 50) {
			maze[position][position+1] = maze[position+1][position] = 1;
		}
		if (sideRightDist > 30) {
			maze[position][position-4] = maze[position-4][position] = 1;
		} 
		if (sideLeftDist > 30) {
			maze[position][position+4] = maze[position+4][position] = 1;
		}
	}
	if (orientation == 2) {
		if (position == 1)
			position = position-1;
		else {
			position = position-4;
			if (USDist > 50) {
				maze[position][position-4] = maze[position-4][position] = 1;
			}
			if (sideRightDist > 30) {
				maze[position][position-1] = maze[position-1][position] = 1;
			} 
			if (sideLeftDist > 30) {
				maze[position][position+1] = maze[position+1][position] = 1;
			}
		}
	}
	if (orientation == 3) {
		position = position-1;
		if (USDist > 50) {
			maze[position][position-1] = maze[position-1][position] = 1;
		}
		if (sideRightDist > 30) {
			maze[position][position+4] = maze[position+4][position] = 1;
		} 
		if (sideLeftDist > 30) {
			maze[position][position-4] = maze[position-4][position] = 1;
		}
	}
}

void floyd() {
	for (int i=0; i<=16; ++i) 
		for (int j=0; j<=16; ++j) {
			paths[i][j] = -1;
	}
	
	for (int k=0; k<=16; k++)
		 for (int i=0; i<=16; i++)
		 	for (int j=0; j<=16; j++)
		 		if (maze[i][k] + maze[k][j] < maze[i][j]) {
		 			maze[i][j] = maze[i][k] + maze[k][j];
		 			paths[i][j] = k;
		 		}
}

void floyd_reconstruction(int i, int j) {
	int k = paths[i][j];
	if (k != -1) {
		floyd_reconstruction(i,k);
	}	
	path *new = p_create();
	new -> square = j;
	p_current -> next = new;
	p_current = p_current -> next;
	//printf("%d\n", j);
} 

int stopper() {
	if (position == 0 && orientation == 2) {
		printf("Phase 1 completed.\n");
		return 0;
	}
	return 1;
}

void correction() {
	if (USDist < 50 && (sideRightDist < 30 || sideLeftDist < 30)) {
		if ((((USDist < 21) || (USDist > 23 && USDist < 50) ) && sideLeftDist < 30)       //US: 21-23 range //30-32
			|| ((sideLeftDist < 19) || (sideLeftDist > 21 && sideLeftDist < 30))) {	      //sideDists: 19-21 range
				//printf("Correction case 1:\n");
				int delta_x = sideLeftDist+10-30;
				int delta_y = USDist+8-30;  
				double alpha = -atan2(delta_x, delta_y) * 180 / M_PI;
				//printf("alpha: %f || delta_x: %d | delta_y: %d || drive_dist: %f\n", alpha, delta_x, delta_y, sqrt(sqr(delta_x)+sqr(delta_y)));
				turn(alpha);
				//usleep(1000000);
				drive(sqrt(sqr(delta_x)+sqr(delta_y)) * CENTIMETER);
				turn(0);
				//usleep(1000000);

				//printf("Post-correction: ");
				get_readings();
			}
			else if ((((USDist < 21) || (USDist > 23 && USDist < 50)) && sideRightDist < 30)
				|| ((sideRightDist < 19) || (sideRightDist > 21 && sideRightDist < 30))) {
					//printf("Correction case 2:\n");
					int delta_x = 30-(sideRightDist+10);
					int delta_y = USDist+8-30;
					double alpha = -atan2(delta_x,delta_y) * 180 / M_PI;
					//printf("alpha: %f || delta_x: %d | delta_y: %d || drive_dist: %f\n", alpha, delta_x, delta_y, sqrt(sqr(delta_x)+sqr(delta_y)));
					turn(alpha);
					drive(sqrt(sqr(delta_x)+sqr(delta_y)) * CENTIMETER);
					turn(0);

					//printf("Post-correction: ");
					get_readings();
			}			
	}
	/*else if (USDist >= 50 && sideLeftDist < 30) {
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
	} */
}

void maze_explorer() {
	while (1) {
		get_readings();
		correction();
		mapper(orientation);
		if (stopper() == 0) break;

	    if (sideLeftDist > 30) {
	    	//printf("Turning -90\n");
	    	turn(-90); 
	    	//printf("Finished Turning -90\n");
	    	orientation = (orientation+3)%4;    
	    }
	    else if (USDist < 40) {
	    	if (sideRightDist < 30) {
				//printf("Turning 180\n");
				turn(180);
				//printf("Finished Turning 180\n");
				orientation = (orientation+2)%4;
			}
			else {
				//printf("Turning 90\n");
				turn(90);
				//printf("Finished Turning 90\n");
				orientation = (orientation+1)%4;
			}
		}
		//printf("Moving ahead\n");
		drive(UNIT-10);								//The -10 is for precision
		//printf("Finished Moving ahead\n");
	}
	turn(180);
}
//////////////////////////////////////////

///////// Phase 2 //////////
void drive_to_target() {
	int speed = 127;
	set_point(target_point_x, target_point_y);
	double heading_r_to_p = atan2(target_point_y - total_d_y, target_point_x - total_d_x);
	angle_change_tracing();
	double error = fmod(final_angle - heading_r_to_p + 2*M_PI, 2*M_PI);
	if (error > M_PI) {
		error -= 2 * M_PI;
	}
	int newtestspeedRight = 0, newtestspeedLeft = 0;
	newtestspeedRight = (int)round(speed + (speed * error));
	newtestspeedLeft = (int)round(speed - (speed * error));
	set_motors(newtestspeedRight, newtestspeedLeft);
}

void race() {
	total_d_x = total_d_y = cm_left = cm_right = 0;
	reset_motor_encoders();
	set_origin();
	current = start;
	while (current -> next != NULL) {
		if (hypotenuse(total_d_y - current ->y, total_d_x - current ->x) < 25.0) {
			current = current -> next;
			if ((current ->x == current -> y) && (current -> y == 200)) {
				printf("Phase 2 completed.\n");
				exit(0);
			}
		}
		else {
			target_point_x = current -> x;
			target_point_y = current -> y;
			//printf("%lf | %lf\n", target_point_x, target_point_y);
			drive_to_target();
		}
	}
	while (hypotenuse(total_d_y - current -> y, total_d_x - current -> x) > 1.0){
		drive_to_target();
	}
}

void trace() {
	int x = 0, y = 30, orientation = 0, left[4] = {-1,4,1,-4}, right[4] = {1,-4,-1,4}, ahead[4] = {4,1,-4,-1};
	p_current = p_start;
	p_current = p_current -> next;
	setLinkedList (0,30);

	while (p_current -> next) {
		int next = p_current -> next -> square;
		int current = p_current -> square;
		if (next - current == ahead[orientation]) {
			if (orientation == 0)
				y += 60;
			else if (orientation == 1)
				x += 60;
			else if (orientation == 2)
				y -= 60;
			else
				x -= 60;
			setLinkedList(x, y);
		}
		else if (next - current == left[orientation]) {
			if (orientation == 0) {
				x -= 10; y += 10;
				setLinkedList(x, y);
				x -= 10; y += 10;
				setLinkedList(x, y);
				x -= 10; y += 10;
				setLinkedList(x, y);
			}
			else if (orientation == 1) {
				x += 10; y += 10;
				setLinkedList(x, y);
				x += 10; y += 10;
				setLinkedList(x, y);
				x += 10; y += 10;
				setLinkedList(x, y);
			}
			else if (orientation == 2) {
				x += 10; y -= 10;
				setLinkedList(x, y);
				x += 10; y -= 10;
				setLinkedList(x, y);
				x += 10; y -= 10;
				setLinkedList(x, y);
			}
			else {
				x -= 10; y -= 10;
				setLinkedList(x, y);
				x -= 10; y -= 10;
				setLinkedList(x, y);
				x -= 10; y -= 10;
				setLinkedList(x, y);
			}
			orientation = (orientation+3)%4;
		}
		else {
			if (orientation == 0) {
				x += 10; y += 10;
				setLinkedList(x, y);
				x += 10; y += 10;
				setLinkedList(x, y);
				x += 10; y += 10;
				setLinkedList(x, y);
			}
			else if (orientation == 1) {
				x += 10; y -= 10;
				setLinkedList(x, y);
				x += 10; y -= 10;
				setLinkedList(x, y);
				x += 10; y -= 10;
				setLinkedList(x, y);
			}
			else if (orientation == 2) {
				x -= 10; y -= 10;
				setLinkedList(x, y);
				x -= 10; y -= 10;
				setLinkedList(x, y);
				x -= 10; y -= 10;
				setLinkedList(x, y);
			}
			else {
				x -= 10; y += 10;
				setLinkedList(x, y);
				x -= 10; y += 10;
				setLinkedList(x, y);
				x -= 10; y += 10;
				setLinkedList(x, y);
			}
			orientation = (orientation+1)%4;
		}
		//printf ("%d\n", p_current -> square);
		p_current = p_current -> next;
	}
	setLinkedList(180,225);
	setLinkedList(200,200);
}
//////////////////////////////////////////

int main() {
	initialisation();
	drive_first_unit();
	maze_explorer();
	floyd();
	floyd_reconstruction(0,16);
	trace();
	race();

	set_motors(0, 0);
	return 0;
}
