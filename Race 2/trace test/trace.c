// Midterm Task Spring 2015
// Code Created by Hekla Helgadottir and Darian Sastre

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "picomms.h"

#define DESIRED_FRONT_DIST 32
#define SPEED 127.0
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

typedef struct coords {
	double x;
	double y;
	struct coords *next;
} coords;

typedef struct path {
	int square;
	struct path *next;
} path;

coords *start, *current, *new;
path *p_start, *p_current;

double sqr(double x){
	return x*x;
}
double hypotenuse(double x, double y){
	return sqrt(sqr(x) + sqr(y));
}

coords *create(void) {
	coords *new;
	new = (coords *)malloc(sizeof(coords));
	new->x = 0.0;
	new->y = 0.0;
	if(new == NULL)
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
	current->next = new;
	current = current->next;
	current->x=total_d_x;
	current->y=total_d_y;
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
	printf("total_d_x: %lf | total_d_y: %lf\n", total_d_x, total_d_y);
	final_angle += delta_angle;
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


void race() {
	current = start;
	while (current->next != NULL) {
		if (hypotenuse(total_d_y - current ->y, total_d_x - current ->x) < 25.0) {
			current = current -> next;
		}
		else {
			target_point_x = current -> x;
			target_point_y = current -> y;
			printf("%lf | %lf\n", target_point_x, target_point_y);
			drive_to_target();
		}
	}
	while (hypotenuse(total_d_y - current->y, total_d_x - current->x) > 1.0){
		drive_to_target();
	}
}

/*void path() {
	/*for (int i=1; i<=73; i++) {
		setLinkedList(0, i);
	}*/
	/*setLinkedList(0,13);
	setLinkedList(30,43);
	setLinkedList(60,73);
	setLinkedList(60,133);
	setLinkedList(90,163);
	setLinkedList(105,148); //
	setLinkedList(120,133);
	setLinkedList(120,103); 
	setLinkedList(120,93);
	/*setLinkedList(120,193);
	setLinkedList(150,223);
	setLinkedList(170,223);
	setLinkedList(171,224);*/
	/*setLinkedList(0,193);
	setLinkedList(30,223);
	setLinkedList(45,208);
	setLinkedList(60,193);
	setLinkedList(90,163);
	setLinkedList(120,193);
	setLinkedList(150,223);
	setLinkedList(165,208);
	setLinkedList(180,193);
	setLinkedList(180,113);
	setLinkedList(180,103);
}

void path2() {
	freopen ("in.txt", "r", stdin);
	double i,j;
	while (!feof(stdin)) {
		scanf("%lf %lf", &i, &j);
		setLinkedList(i, j);
	}
	fclose (stdin);
} 

void print() {
	current = start;
	while (current) {
		printf("%f | %f\n",current->x,current->y);
		current = current->next;
	} 
} 

void circle() {
	for (int i=-30; i<=30; i++)
		for (int j=-30; j<=30; j++)
			if (sqr(i-0) + sqr(j-0) == 900) {
				printf("%d, %d\n",i, j);
				setLinkedList(i, j);
			}
}*/

void readd() {
	freopen ("in.txt", "r", stdin);
	int i;
	while (!feof(stdin)) {
		scanf("%d", &i);
		//printf("%d\n", i);
		path *new = p_create();
		new -> square = i;
		p_current -> next = new;
		p_current = p_current -> next;
	}
	fclose (stdin);
}

void trace() {
	int x = 0, y = 13, orientation = 0, left[4] = {-1,4,1,-4}, right[4] = {1,-4,-1,4}, ahead[4] = {4,1,-4,-1};
	p_current = p_start;
	p_current = p_current -> next;
	setLinkedList (0,13);

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
				x -= 15; y += 15;
				setLinkedList(x, y);
				x -= 15; y += 15;
				setLinkedList(x, y);
			}
			else if (orientation == 1) {
				x += 15; y += 15;
				setLinkedList(x, y);
				x += 15; y += 15;
				setLinkedList(x, y);

			}
			else if (orientation == 2) {
				x += 15; y -= 15;
				setLinkedList(x, y);
				x += 15; y -= 15;
				setLinkedList(x, y);
			}
			else {
				x -= 15; y -= 15;
				setLinkedList(x, y);
				x -= 15; y -= 15;
				setLinkedList(x, y);
			}
			orientation = (orientation+3)%4;
		}
		else {
			if (orientation == 0) {
				x += 15; y += 15;
				setLinkedList(x, y);
				x += 15; y += 15;
				setLinkedList(x, y);
			}
			else if (orientation == 1) {
				x += 15; y -= 15;
				setLinkedList(x, y);
				x += 15; y -= 15;
				setLinkedList(x, y);
			}
			else if (orientation == 2) {
				x -= 15; y -= 15;
				setLinkedList(x, y);
				x -= 15; y -= 15;
				setLinkedList(x, y);
			}
			else {
				x -= 15; y += 15;
				setLinkedList(x, y);
				x -= 15; y += 15;
				setLinkedList(x, y);
			}
			orientation = (orientation+1)%4;
		}
		//printf ("%d\n", p_current -> square);
		p_current = p_current -> next;
	}
}

void printt() {
	current = start;
	while (current) {
		printf("%lf | %lf\n", current->x, current->y);
		current = current -> next;
	}
}

int main() {
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(1, -45);

	p_current = p_create();
	p_start = p_current;

	current = create();
	start = current;

	readd();
	trace();
	race();

	set_motors(0, 0);
	return 0;
}
