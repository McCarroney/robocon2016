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
	const int shutdown = 21;

	//LED for buttery check
	const int power_check = 13;

	//set maximum PWM
	const int max_pwm = 200;

	//急発進と急制動を抑制…したい
	/*double temp_ly = 0;
	double temp_ry = 0;
	bool rapid_flag = 0;
	int extend_ly = 0;
	int extend_ry = 0;
	bool yuruyaka_ly = 0;
	bool yuruyaka_ry = 0;*/

	//spining motor
	bool cw = 0, ccw = 0;

	//130motor
	const int motor1_IN1 = 8;
	const int motor1_IN2 = 9;
	const int motor2_IN1 = 7;
	const int motor2_IN2 = 10;

	//solenoid valve
	const int solenoid_valve_r = 2;
	const int solenoid_valve_l = 3;
	bool valve_gate_r = 0;
	bool valve_gate_l = 0;
	bool sending_check_valve_r = 0;
	bool sending_check_valve_l = 0;
	//bool press_flag = 0;

	//vacuum pump
	bool sending_check_pump_0 = 1;
	bool sending_check_pump_1 = 1;

	//electromagnet
	bool magnet_gate = 0;
	bool sending_check_magnet = 0;

	//for automatic control
	const int over_limit = 17;
	const int under_limit = 18;
	const int hold_a_detect = 27;
	const int open_a_limit = 22;
	const int hight_a_detect = 23;
	const int hold_b_detect = 24;
	const int open_b_limit = 25;
	const int hight_b_detect = 5;
	const int hight_200_detect = 12;

	//command
	bool command = 0;
	//bool checking[10];
	//bool checking2[5];
	//for(int j=0;j<10;j++) checking[j] = 0;
	//for(int k=0;k<5;k++) checking2[k] = 0;
	
	//for dual mode
	bool dual_flag = 0;

	//for cross mode
	bool cross_flag = 0;	

	//for unison mode
	bool unison_flag = 0;

	//for slow mode
	bool magnification = 1;
	bool magni_1 = 0;
	bool magni_2 = 0;

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

	pinMode (shutdown,INPUT);
	pinMode (power_check,OUTPUT);
	pinMode (solenoid_valve_r,OUTPUT);
	pinMode (solenoid_valve_l,OUTPUT);
	pinMode (motor1_IN1,OUTPUT);
	pinMode (motor1_IN2,OUTPUT);
	pinMode (motor1_IN1,OUTPUT);
	pinMode (motor2_IN2,OUTPUT);
	pinMode (over_limit,INPUT);
	pinMode (under_limit,INPUT);
	pinMode (hold_a_detect,INPUT);
	pinMode (open_a_limit,INPUT);
	pinMode (hight_a_detect,INPUT);
	pinMode (hold_b_detect,INPUT);
	pinMode (open_b_limit,INPUT);
	pinMode (hight_b_detect,INPUT);
	pinMode (hight_200_detect,INPUT);


	pullUpDnControl (over_limit,PUD_DOWN);
	pullUpDnControl (under_limit,PUD_DOWN);
	pullUpDnControl (hold_a_detect,PUD_DOWN);
	pullUpDnControl (open_a_limit,PUD_DOWN);
	pullUpDnControl (hight_a_detect,PUD_DOWN);
	pullUpDnControl (hold_b_detect,PUD_DOWN);
	pullUpDnControl (open_b_limit,PUD_DOWN);
	pullUpDnControl (hight_b_detect,PUD_DOWN);
	pullUpDnControl (hight_200_detect,PUD_DOWN);

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

	double left_front = 0;
	double left_rear = 0;
	double right_front = 0;
	double right_rear = 0;	

	//void cross_control (bool*,bool*);
	void auto_constract (void);

	//main routine
	UPDATELOOP (controller,!(controller.button(START)&&controller.button(CROSS))){

		//emergency stop
		if(controller.press(SELECT)){
			sm.send(255,255,0);
			cout << "Emergency stop" << endl;
			UPDATELOOP(controller,!controller.press(SELECT));
		}
		
		//dual mode
		if (controller.press(LEFT_STICK)) {
			if (!dual_flag) {
				dual_flag = 1;
				cout << "Dual mode ON" << endl;
			}
			else {
				dual_flag = 0;
				cout << "Dual mode OFF" << endl;
			}
		}

		//slow mode
		if (controller.press(L1)) magni_1 = 0.5;
		if (controller.release(L1)) magni_1 = 1;
		if (controller.press(R1)) magni_2 = 0.25;
		if (controller.release(R1)) magni_2 = 1;
		magnification = magni_1*magni_2;

		//control spining
		if (controller.press(R2)) {cw = 1;}
		if (controller.release(R2)) {cw = 0;}

		if (controller.press(L2)) {ccw = 1;}
		if (controller.release(L2)) {ccw = 0;}

		if (ccw){
			cout << "ccw\t" << magnification*controller.stick(LEFT_T) << endl;
			sm.send(14,2,magnification*controller.stick(LEFT_T));
		}

		else if (cw){
			cout << "cw\t" << magnification*controller.stick(RIGHT_T) << endl;
			sm.send(14,2,-magnification*controller.stick(RIGHT_T));
		}

		//command flag
		if (controller.button(L1)&&controller.button(R1)) command = 1;
		else {
			command = 0;
		}

		//change to cross control mode
		if (command&&controller.press(UP)) {
			if(!cross_flag) {
				cross_flag = 1;
				cout << "Cross mode ON" << endl;
			}
			else {
				cross_flag = 0;
				cout << "Cross mode OFF" << endl;
			}
		}
		
		//change to unison control mode
		if (command&&controller.press(CIRCLE)) {
			if (!unison_flag) {
				unison_flag = 1;
				cout << "Unison mode ON" << endl;
			}
			else {
				unison_flag = 0;
				cout << "Unison mode OFF" << endl;
			}
		}

		//start automated constraction
		if (command&&controller.press(CROSS)) auto_constract();
		
		//release Naruto bridge
		if (command&&controller.press(TRIANGLE)) {
			digitalWrite(motor1_IN1,1);
			digitalWrite(motor1_IN2,0);
			cout << "Motor1 ON" << endl;
		}
		if (controller.release(TRIANGLE)) {
			digitalWrite(motor1_IN1,0);
			digitalWrite(motor1_IN2,0);
		}

		//release Akashi bridge
		if (command&&controller.press(SQUARE)) {
			digitalWrite(motor2_IN1,1);
			digitalWrite(motor2_IN2,0);
			cout << "Motor2 ON" << endl;
		}
		if (controller.release(SQUARE)) {
			digitalWrite(motor2_IN1,0);
			digitalWrite(motor2_IN2,0);
		}
		
		//control electromagnet
		if (command&&controller.press(RIGHT)){
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
	
		//control solenoid valve r
		if (!command&&controller.press(TRIANGLE)){
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
		if (!command&&controller.press(SQUARE)){
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

		//control vacuum pump
		if (sending_check_pump_0&&(valve_gate_r||valve_gate_l)){
			sm.send(7,2,max_pwm,false);
			cout << "Pump ON" << endl;		
			sending_check_pump_0 = 0;
			sending_check_pump_1 = 1;
		}
		if (sending_check_pump_1&&(!valve_gate_r&&!valve_gate_l)) {
			sm.send(7,2,0,false);
			cout << "Pump OFF" << endl;
			sending_check_pump_1 = 0;
			sending_check_pump_0 = 1;
		}

		if (!command&&!digitalRead(over_limit)&&controller.press(UP)){
			if (digitalRead(hight_200_detect)){
				//catcher of above up
				sm.send(16,2,100);
				cout << "Catcher of above UP" << endl;
			}
			//catcher of below up
			sm.send(16,3,120);
			cout << "Catcher of below UP" << endl;
		}
		if (!command&&!digitalRead(under_limit)&&controller.press(DOWN)){
			if (digitalRead(hight_200_detect)){
				//catcher of below down
				sm.send(16,2,-100);
				cout << "Catcher of below DOWN" << endl;
			}
			//catcher of above down
			sm.send(16,3,-120);
			cout << "Catcher of above DOWN" << endl;
		}
		if (controller.release(UP)||controller.release(DOWN)||digitalRead(over_limit)||digitalRead(under_limit)) {
			sm.send(16,2,0);
			sm.send(16,3,0);
		}

		if (!command&&controller.press(RIGHT)&&!digitalRead(hold_a_detect)){
			//catcher of above close
			sm.send(15,2,magnification*max_pwm/2,false);
			cout << "Catcher of above CLOSE" << endl;
		}
		if (!command&&controller.press(RIGHT)&&!digitalRead(hold_b_detect)){
			//catcher of below close
			sm.send(15,3,magnification*max_pwm/2,false);
			cout << "Catcher of below CLOSE" << endl;
		}
		if (!command&&controller.press(LEFT)&&!digitalRead(open_a_limit)){
			//catcher of below open
			sm.send(15,2,-magnification*max_pwm/2,false);
			cout << "Catcher of above OPEN" << endl;
		}
		if (!command&&controller.press(LEFT)&&!digitalRead(open_b_limit)){
			//catcher of below open
			sm.send(15,3,-magnification*max_pwm/2,false);
			cout << "Catcher of below OPEN" << endl;
		}
		if (controller.release(RIGHT)||controller.release(SQUARE)) {
			sm.send(15,2,0);
			sm.send(15,3,0);
		}
		if (digitalRead(hold_a_detect)||digitalRead(open_a_limit)) sm.send(15,2,0);
		if (digitalRead(hold_b_detect)||digitalRead(open_b_limit)) sm.send(15,3,0);

		//roger arm up and down
		if(!command&&controller.press(CIRCLE)){
			sm.send(14,2,magnification*max_pwm);
			cout << "Roger UP" << endl;
		}
		if(!command&&controller.press(CROSS)){
			sm.send(14,2,-magnification*max_pwm);
			cout << "Roger DOWN" << endl;
		}
		if(controller.release(CIRCLE)||controller.release(CROSS)) sm.send(14,2,0);

		//control ashimawari
		
		//for yuruyaka control
		//if(controller.press(L1)) rapid_flag = 1;
		//if(controller.release(L1)) rapid_flag = 0;
		//temp_ly = left_y;
		//temp_ry = right_y;

		if(!cross_flag){
		//control left side motor
		left_y = controller.stick(LEFT_Y);
		left_x = controller.stick(LEFT_X);

		left_y = -200*left_y/128;
		left_x = 200*left_x/128;

		left_front = (left_x*sqrt(2.)/2)+(left_y*sqrt(2.)/2);
		left_rear = (left_x*sqrt(2.)/2)-(left_y*sqrt(2.)/2);

		//if(left_y>max_pwm)left_y=max_pwm;
		//if(left_x>max_pwm)left_x=max_pwm;

		//printf("%lf\n",left_y);

		//control right side motor
		right_y = controller.stick(RIGHT_Y);
		right_x = controller.stick(RIGHT_X);

		right_y = -200*right_y/128;
		right_x = 200*right_x/128;

		right_rear = (right_x*sqrt(2)/2)+(right_y*sqrt(2)/2);
		right_front = (right_x*sqrt(2)/2)-(right_y*sqrt(2)/2);

		//if(right_y>max_pwm)right_y=max_pwm;
		//if(right_x>max_pwm)right_x=max_pwm;
	
		}
		else{
			if(controller.press(UP)){
				left_front = max_pwm;
				left_rear = -max_pwm;
				right_front = -max_pwm;
				right_rear = max_pwm;
			}
			if(controller.press(DOWN)){
				left_front = -max_pwm;
				left_rear = max_pwm;
				right_front = max_pwm;
				right_rear = -max_pwm;
			}
			if(controller.press(RIGHT)){
				left_front = max_pwm;
				left_rear = max_pwm;
				right_front = max_pwm;
				right_rear = max_pwm;
			}
			if(controller.press(LEFT)){
				left_front = -max_pwm;
				left_rear = -max_pwm;
				right_front = -max_pwm;
				right_rear = -max_pwm;
			}
			if(controller.release(UP)||controller.release(DOWN)||controller.release(RIGHT)||controller.release(LEFT)){
				left_front = 0;
				left_rear = 0;
				right_front = 0;
				right_rear = 0;
			}
		}

		//printf("right_x:%lf\t",right_rear);
		//printf("right_y:%lf\n",right_front);
		//printf("left_x:%lf\t",left_rear);
		//printf("left_y:%lf\n\n",left_front);	

		if(dual_flag){
			sm.send(12,2,left_front*magnification,false);
			sm.send(12,3,-left_rear*magnification,false);
			sm.send(13,2,-left_front*magnification,false);
			sm.send(13,3,left_rear*magnification,false);
		}		
	
		else{
			sm.send(12,2,left_front*magnification,false);
			sm.send(12,3,-left_rear*magnification,false);
			sm.send(13,3,right_front*magnification,false);
			sm.send(13,2,-right_rear*magnification,false);
		}
	}
	sm.send(255,255,0,false);
	digitalWrite(power_check,0);
	return 0;	
}

/*void cross_control (bool *cross_flag,bool *dual_flag){
	cout << "コナミコマンドw" << endl;
	*cross_flag = 1;
	*dual_flag = 1;
}*/

void auto_constract(){
	cout << "Constracting now..." << endl;
}
