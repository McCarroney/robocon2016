#include <iostream>
#include <wiringPi.h>
#include <RobotUtil.hpp>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstdio>
#include <unistd.h>

using namespace std;
using namespace rbutil;

char* ConfigurationFileName = {(char*)"MotorConfig"};

int main(void){
	
	int wiringPiSetup();
	wiringPiSetupSys();

	//switch for shutdown
	int shutdown = 21;

	//buttery checker
	int power_check = 13;

	//set maximum pwm
	int max_pwm = 200;

	//set communication to MDD
	ScrpMaster sm;
	try{
		sm.init();
	}
	catch(...){
		return-1;
	}

	//check controller connecting
	Ds3Read controller;
	if(!controller.isConnected()){
		cout << "Couldn't connected Dualshock3." << endl;
		return -1;
	}

	pinMode(shutdown,INPUT);
	pinMode(power_check,OUTPUT);

	digitalWrite(power_check,1);

	//load configuration for motors
	const int motor_num = 4;
	MotorDataFormat MDF[motor_num];
	loadMotorSetting(ConfigurationFileName,MDF,motor_num);

	Motor left_front (MDF[0],sm);
	Motor right_front(MDF[1],sm);
	Motor right_rear (MDF[2],sm);
	Motor left_rear  (MDF[3],sm);

	//main routine
	UPDATELOOP (controller,!(controller.button(START)&&controller.button(CROSS))){

		//emargency stop
		if (controller.press(SELECT)){
			sm.send(255,255,0);
			UPDATELOOP (controller,!controller.press(SELECT)&&!controller.button(START))
			digitalWrite(power_check,0);
		}

		//switch dual mode
		int dual_mode = false;

		if (controller.button(R2))
			dual_mode = true;

		//regulation
		double regulation =1;
		if(controller.button(L2)==true)regulation = 0.2;


		//control left side motor
		double left_y = 0;
		double left_x = 0;

		left_y = controller.stick(LEFT_Y);
		left_x = -1*controller.stick(LEFT_X);

		if(left_y>max_pwm)left_y=max_pwm;
		if(left_x>max_pwm)left_x=max_pwm;

		left_front.spin(left_y+left_x);
		left_rear.spin (left_y-left_x);

		//control right side motor
		double right_y = 0;
		double right_x = 0;

		right_y = controller.stick(RIGHT_Y);
		right_x = -1*controller.stick(RIGHT_X);

		if(right_y>max_pwm)right_y=max_pwm;
		if(right_x>max_pwm)right_x=max_pwm;

		right_front.spin(right_y+right_x);
		right_rear.spin (right_y-right_x);
	
		return 0;
	}
}
