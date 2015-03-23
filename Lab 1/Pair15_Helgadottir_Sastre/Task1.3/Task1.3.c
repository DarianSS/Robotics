#include <stdio.h>
#include "picomms.h"
#include <unistd.h>
#define degreeCorner 218
#define degreeStraight 1146

int main() 
{
	connect_to_robot();
	initialize_robot();
	int speed = 20;
	int i, leftMotor = 0, rightMotor = 0;
	for(i = 1; i <= 4; i++)
	{
		int straight = i*degreeStraight+(i-1)*degreeCorner;
		int turn = i*(degreeStraight+degreeCorner);
		while (leftMotor<straight)
		{
			set_motor(LEFT, speed);
			set_motor(RIGHT, speed);
			get_motor_encoders(&leftMotor, &rightMotor);
			log_trail();
		}
		if(i < 4)
		{	
			while(leftMotor<turn)
			{
				set_motor(LEFT, speed);
				set_motor(RIGHT, -speed);
				get_motor_encoders(&leftMotor, &rightMotor);
				log_trail();
			}
		}
	}
	set_motors(0,0);
}