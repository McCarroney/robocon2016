#include <iostream>
#include <wiringPi.h>
#include <RobotUtil.hpp>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstdio>
#include <unistd.h>
#include <stdio.h>

using namespace std;
using namespace rbutil;

//char* ConfigurationFileName = {(char*)"MotorConfig"};

const int excitation[4] = {17,18,22,23};

int main(void){
	
	int wiringPiSetup();
	wiringPiSetupSys();

	//switch for shutdown
	int shutdown = 21;

	//buttery checker
	int power_check = 13;

	//set maximum pwm
	int max_pwm = 200;

	//stepping motor
	unsigned int counter1 = 0, counter2 = 0;
	int tern1 = 0, tern2 = 1;
	bool cw = 0, ccw = 0;

	//solenoid valve
	bool valve_gate = 0;
	bool sending_check = 0;
	//bool press_flag = 0;

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
	}else {cout << "Connected." << endl;}

	pinMode(shutdown,INPUT);
	pinMode(power_check,OUTPUT);
	for (int i=0;i<4;i++){
		pinMode (excitation[i],OUTPUT);
		digitalWrite (excitation[i],0);
	}

	//digitalWrite(excitation[0],1);
	digitalWrite(power_check,1);

	/*//load configuration for motors
	const int motor_num = 4;
	MotorDataFormat MDF[motor_num];
	loadMotorSetting(ConfigurationFileName,MDF,motor_num);

	Motor left_front (MDF[0],sm);
	Motor right_front(MDF[1],sm);
	Motor right_rear (MDF[2],sm);
	Motor left_rear  (MDF[3],sm);*/

	//motor configuration
	Motor left_front (11,2,1.0,sm);
	Motor right_front (12,2,1.0,sm);
	Motor right_rear (13,2,1.0,sm);
	Motor left_rear (14,2,1.0,sm);

	double left_y = 0;
	double left_x = 0;
	double right_y = 0;
	double right_x = 0;

	sm.send(11,2,100,false);

	//main routine
	UPDATELOOP (controller,!controller.button(START,true)){

		if (controller.press(RIGHT)) {cw = 1; /*cout << "ccw" << endl;*/}
		if (controller.release(RIGHT)) {cw = 0; counter1 = 0;}

		if (controller.press(LEFT)) {ccw = 1; /*cout << "cw" << endl;*/}
		if (controller.release(LEFT)) {ccw = 0; counter2 = 0;}

		if (ccw){
			cout << "ccw" << endl;
			if (counter1 > 10){
				//cout << "ccw" << endl;
				digitalWrite (excitation[tern1], 0);
				tern1 ++;
				tern2 ++;
				if (tern1 > 3)tern1 = 0;
				if (tern2 > 3)tern2 = 0;
				digitalWrite (excitation[tern1], 1);
				digitalWrite (excitation[tern2], 1);
			}else counter1 ++;
		}

		else if (cw){
			cout << "cw" << endl;
			if (counter2 > 10){
				//cout << "cw" << endl;
				digitalWrite (excitation[tern2], 0);
				tern1 --;
				tern2 --;
				if (tern1 < 0)tern1 = 3;
				if (tern2 < 0)tern2 = 3;
				digitalWrite (excitation[tern1], 1);
				digitalWrite (excitation[tern2], 1);
			}else counter2 ++;
		}

		if (controller.press(TRIANGLE)){
			if (valve_gate){
				valve_gate = 0;	
				cout << "OFF" << endl;
			}else{
				valve_gate = 1;
				cout << "ON" << endl;
			}
			sending_check = 1;
		}
		
		if (sending_check){
			if (valve_gate) sm.send(7,2,200,false);
			else sm.send(7,2,0,false);
			sending_check = 0;
		}

		//control left side motor
		left_y = -1*controller.stick(LEFT_Y);
		left_x = controller.stick(LEFT_X);

		if(left_y>max_pwm)left_y=max_pwm;
		if(left_x>max_pwm)left_x=max_pwm;

		//printf("%lf\n",left_y+left_x);
		
		if (fabs(left_y)){
			cout << "spin" << endl;
		//	left_front.spin(left_y);
		}
		//left_rear.spin (left_y);

		//control right side motor
		right_y = -1*controller.stick(RIGHT_Y);
		right_x = controller.stick(RIGHT_X);

		if(right_y>max_pwm)right_y=max_pwm;
		if(right_x>max_pwm)right_x=max_pwm;

		//printf("%lf\n",right_y+right_x);

		/*right_front.spin(right_y+right_x);
		right_rear.spin (right_y-right_x);*/
	
		sm.send(11,2,left_y,false);

		cout << "LOOP" << endl;	

	}
	return 0;	
}
