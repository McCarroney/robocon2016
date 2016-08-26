#include <iostream>
#include <wiringPi.h>
//#include <RobotUtil.hpp>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstdio>
#include <unistd.h>
#include <stdio.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"

using namespace std;
//using namespace rbutil;
using namespace RPDS3;
using namespace RPMS;

//char* ConfigurationFileName = {(char*)"MotorConfig"};

const int excitation[4] = {17,18,22,23};

int main(void){
	
	//setup GPIO pin
	int wiringPiSetup();
	wiringPiSetupSys();

	//switch for shutdown
	int shutdown = 21;

	//LED for buttery check
	int power_check = 13;

	//set maximum PWM
	int max_pwm = 200;

	//急発進と急制動を抑制…したい
	/*double temp_ly = 0;
	double temp_ry = 0;
	bool rapid_flag = 0;
	int extend_ly = 0;
	int extend_ry = 0;
	bool yuruyaka_ly = 0;
	bool yuruyaka_ry = 0;*/

	//stepping motor
	unsigned int counter1 = 0, counter2 = 0;
	int tern1 = 0, tern2 = 1;
	bool cw = 0, ccw = 0;

	//solenoid valve
	int solenoid_valve_r = 2;
	int solenoid_valve_l = 3;
	bool valve_gate_r = 0;
	bool valve_gate_l = 0;
	bool sending_check_valve_r = 0;
	bool sending_check_valve_l = 0;
	//bool press_flag = 0;

	//vacuum pump
	bool pump_gate = 0;
	bool sending_check_pump = 0;

	//electromagnet
	bool magnet_gate = 0;
	bool sending_check_magnet = 0;

	//set communication to MDD
	//ScrpMaster sm;
	MotorSerial sm;
	try{
		sm.init();
	}
	catch(...){
		return-1;
	}

	//check controller connecting
	//Ds3Read controller;
	DualShock3 controller;
	if(!controller.connectedCheck()){
		cout << "Couldn't connected Dualshock3." << endl;
		return -1;
	}

	pinMode(shutdown,INPUT);
	pinMode(power_check,OUTPUT);
	for (int i=0;i<4;i++){
		pinMode (excitation[i],OUTPUT);
		digitalWrite (excitation[i],0);
	}
	pinMode(solenoid_valve_r,OUTPUT);
	pinMode(solenoid_valve_l,OUTPUT);

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
	//Motor left_front (11,2,1.0,sm);
	//Motor right_front (12,2,1.0,sm);
	//Motor right_rear (13,2,1.0,sm);
	//Motor left_rear (14,2,1.0,sm);

	//mecha wheel
	double left_y = 0;
	double left_x = 0;
	double right_y = 0;
	double right_x = 0;

	//main routine
	UPDATELOOP (controller,!(controller.button(START)&&controller.button(CROSS))){

		//emergency stop
		if(controller.press(SELECT)){
			sm.send(255,255,0);
			cout << "Emergency stop" << endl;
			UPDATELOOP(controller,!controller.press(SELECT));
		}
		
		//control stepping motor
		if (controller.press(R2)) {cw = 1; /*cout << "ccw" << endl;*/}
		if (controller.release(R2)) {cw = 0; counter1 = 0;}

		if (controller.press(L2)) {ccw = 1; /*cout << "cw" << endl;*/}
		if (controller.release(L2)) {ccw = 0; counter2 = 0;}

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

		//control electromagnet
		if (controller.press(TRIANGLE)){
			if (magnet_gate){
				magnet_gate = 0;	
			}else{
				magnet_gate = 1;
			}
			sending_check_magnet = 1;
		}
		
		if (sending_check_magnet){
			if (magnet_gate){
				sm.send(7,3,max_pwm,false);
				cout << "Magnet ON" << endl;
			}
			else{
				sm.send(7,3,0,false);
				cout << "Magnet OFF" << endl;
			}
			sending_check_magnet = 0;
		}

		//control vacuum pump
		if (controller.press(CROSS)){
			if (pump_gate){
				pump_gate = 0;	
			}else{
				pump_gate = 1;
			}
			sending_check_pump = 1;
		}
		
		if (sending_check_pump){
			if (pump_gate){
				sm.send(7,2,-max_pwm,false);
				cout << "Pump ON" << endl;
			}
			else{
				sm.send(7,2,0,false);
				cout << "Pump OFF" << endl;
			}
			sending_check_pump = 0;
		}

		//control solenoid valve r
		if (controller.press(CIRCLE)){
			if (valve_gate_r){
				valve_gate_r = 0;	
			}else{
				valve_gate_r = 1;
			}
			sending_check_valve_r = 1;
		}
		
		if (sending_check_valve_r){
			if (valve_gate_r){
				digitalWrite(solenoid_valve_r,1);
				cout << "Valve_R ON" << endl;
			}
			else{
				digitalWrite(solenoid_valve_r,0);
				cout << "Valve_R OFF" << endl;
			}
			sending_check_valve_r = 0;
		}

		//control solenoid valve l
		if (controller.press(SQUARE)){
			if (valve_gate_l){
				valve_gate_l = 0;	
			}else{
				valve_gate_l = 1;
			}
			sending_check_valve_l = 1;
		}
		
		if (sending_check_valve_l){
			if (valve_gate_l){
				digitalWrite(solenoid_valve_l,1);
				cout << "Valve_L ON" << endl;
			}
			else{
				digitalWrite(solenoid_valve_l,0);
				cout << "Valve_L OFF" << endl;
			}
			sending_check_valve_l = 0;
		}

		//up and down
		if(controller.press(UP)){
			sm.send(13,2,max_pwm);
			cout << "UP" << endl;
		}
		if(controller.press(DOWN)){
			sm.send(13,2,-max_pwm);
		}
		if(controller.release(UP)||controller.release(DOWN)) sm.send(13,2,0);

		//control ashimawari
		
		//dual mode
		bool dual_flag = 0;
		if(controller.button(R1)) dual_flag = 1;

		//slow mode
		double magnification = 1;
		if(controller.button(L1)) magnification = 0.25; 

		//for yuruyaka control
		//if(controller.press(L1)) rapid_flag = 1;
		//if(controller.release(L1)) rapid_flag = 0;
		//temp_ly = left_y;
		//temp_ry = right_y;

		//control left side motor
		left_y = controller.stick(LEFT_Y);
		left_x = controller.stick(LEFT_X);

		left_y = -25*left_y/16;
		left_x = 25*left_x/16;

		left_y = (left_x*sqrt(2)/2)+(left_y*sqrt(2)/2);
		left_x = (left_x*sqrt(2)/2)-(left_y*sqrt(2)/2);

		//if(left_y>max_pwm)left_y=max_pwm;
		//if(left_x>max_pwm)left_x=max_pwm;
	
		/*if(left_y==0&&!rapid_flag){
			yuruyaka_ly = 1;
		}
		else yuruyaka_ly = 0;	

		if(yuruyaka_ly){
			if(extend_ly > 15){
				left_y = temp_ly/2;
				temp_ly = left_y;
				if(left_y < 10)left_y = 0;
				extend_ly = 0;
			}else{
				left_y = temp_ly;
				extend_ly ++;
			}
		}*/

		//printf("%lf\n",left_y);

		//control right side motor
		right_y = -25*controller.stick(RIGHT_Y)/16;
		right_x = 25*controller.stick(RIGHT_X)/16;

		right_y = -25*right_y/16;
		right_x = 25*right_x/16;

		right_y = (right_x*sqrt(2)/2)+(right_y*sqrt(2)/2);
		right_x = (right_x*sqrt(2)/2)-(right_y*sqrt(2)/2);

		//if(right_y>max_pwm)right_y=max_pwm;
		//if(right_x>max_pwm)right_x=max_pwm;
	
		/*if(right_y==0&&!rapid_flag){
			yuruyaka_ry = 1;
		}
		else yuruyaka_ry = 0;	

		if(yuruyaka_ry){
			if(extend_ry > 15){
				right_y = temp_ry/2;
				temp_ry = right_y;
				if(right_y < 10)right_y = 0;
				extend_ry = 0;
			}else{
				right_y = temp_ry;
				extend_ry ++;
			}
		}*/

		printf("right_x:%lf\t",right_x);
		printf("right_y:%lf\n",right_y);
		printf("left_x:%lf\t",left_x);
		printf("left_y:%lf\n\n",left_y);

		if(dual_flag){
			sm.send(11,2,left_y*magnification,false);
			sm.send(11,3,left_x*magnification,false);
			sm.send(12,2,left_y*magnification,false);
			sm.send(12,3,left_x*magnification,false);
		}		
	
		else{
			sm.send(11,2,left_y*magnification,false);
			sm.send(11,3,left_x*magnification,false);
			sm.send(12,2,right_y*magnification,false);
			sm.send(12,3,right_x*magnification,false);
		}
	}
	sm.send(255,255,0,false);
	digitalWrite(power_check,0);
	return 0;	
}
