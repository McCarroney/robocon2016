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

//switch for shutdown
const int shutdown = 21;

//LED for buttery check
const int power_check = 13;

//LED for indicater
const int arm_check = 2;
const int cross_check = 3;
const int pump_check = 11;
const int magnet_check = 20;

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

//for spining motor
bool cw = 0, ccw = 0;

//130motor for release bridge
const int motor1_IN1 = 8;
const int motor1_IN2 = 9;
const int motor2_IN1 = 7;
const int motor2_IN2 = 10;

//solenoid valve
const int solenoid_valve_r = 19;
const int solenoid_valve_l = 26;
bool sending_check_valve_r = 0;
bool sending_check_valve_l = 0;
unsigned int start_time_r = 0;
unsigned int start_time_l = 0;
char valve_switch = 1;

//vacuum pump
bool pump_gate = 0;
bool sending_check_pump = 0;
/*bool sending_check_pump_0 = 1;
bool sending_check_pump_1 = 1;*/

//electromagnet
bool magnet_gate = 0;
bool sending_check_magnet = 0;

//for 1-2constract arm
bool sending_check_up = 0;
bool sending_check_down = 0;

//for automatic control
const int over_limit = 17;
const int under_limit = 18;
const int hold_a_detect = 24;
const int open_a_limit = 25;
const int hight_a_detect = 5;
const int hold_b_detect = 27;
const int open_b_limit = 22;
const int hight_b_detect = 23;
const int hight_200_detect = 6;
bool over_limit_flag = 0;
bool under_limit_flag = 0;
bool open_a_limit_flag = 0;
bool open_b_limit_flag = 0;
bool hold_a_detect_flag = 0;
bool hold_b_detect_flag = 0;

//for command mode
bool command_flag = 0;
//bool checking[10];
//bool checking2[5];
//for(int j=0;j<10;j++) checking[j] = 0;
//for(int k=0;k<5;k++) checking2[k] = 0;
	
//for dual mode
bool dual_flag = 0;

//for cross mode
bool cross_flag = 0;	

//for link mode
bool link_flag = 0;

//for arm mode
bool arm_flag = 0;
bool midover_limit_flag = 0;
bool midunder_limit_flag = 0;
bool sending_check_uup = 0;
bool sending_check_udown = 0;

//for slow mode
float magnification = 1;
float magni_1 = 1;
float magni_2 = 1;

MotorSerial sm;
DualShock3 controller;
	
int main(void){
	
	//setup GPIO pin
	int wiringPiSetup();
	wiringPiSetupSys();

	//set communication to MDD
	//ScrpMaster sm;
	try{
		sm.init();
	}
	catch(...){
		return-1;
	}

	//check controller connecting
	//Ds3Read controller;
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
	pinMode (motor2_IN1,OUTPUT);
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
	pinMode (arm_check,OUTPUT);
	pinMode (cross_check,OUTPUT);
	pinMode (pump_check,OUTPUT);
	pinMode (magnet_check,OUTPUT);

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
	digitalWrite(solenoid_valve_r,0);
	digitalWrite(solenoid_valve_l,1);
	digitalWrite(motor1_IN1,0);
	digitalWrite(motor1_IN2,0);
	digitalWrite(motor2_IN1,0);
	digitalWrite(motor2_IN2,0);
	digitalWrite(arm_check,0);
	digitalWrite(cross_check,0);
	digitalWrite(pump_check,0);
	digitalWrite(magnet_check,0);

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

	//mechanum wheel
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

	//unsigned int loop_time = 0;
	
	//main routine
	UPDATELOOP (controller,!(controller.button(START)&&controller.button(CROSS))){
		//digitalWrite(arm_check,1);	
		//cout << clock()-loop_time << endl;
		//loop_time = clock();
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
			}else {
				dual_flag = 0;
				cout << "Dual mode OFF" << endl;
			}
		}

		//slow mode
		if (controller.press(L1)) magni_1 = 0.5;
		if (controller.release(L1)) magni_1 = 1;
		if (controller.press(R1)) magni_2 = 0.175;
		if (controller.release(R1)) magni_2 = 1;
		magnification = magni_1*magni_2;

		//control spining of box catcher
		if (controller.press(R2)) {cw = 1;}
		if (controller.release(R2)) {
			cw = 0;
			sm.send(10,2,0);
		}

		if (controller.press(L2)) {ccw = 1;}
		if (controller.release(L2)) {
			ccw = 0;
			sm.send(10,2,0);
		}

		if (ccw){

			//cout << "ccw\t" << magnification*controller.stick(LEFT_T) << endl;
			sm.send(10,2,magnification*controller.stick(LEFT_T)/5);
		}
		else if (cw){
			//cout << "cw\t" << magnification*controller.stick(RIGHT_T) << endl;
			sm.send(10,2,-magnification*controller.stick(RIGHT_T)/5);
		}

		//change to command mode
		if (controller.button(L1)&&controller.button(R1)) command_flag = 1;
		else command_flag = 0;

		//change to cross control mode
		if (command_flag&&controller.press(UP)) {
			arm_flag = 0;
			digitalWrite(arm_check,0);
			if(!cross_flag) {
				cross_flag = 1;
				cout << "Cross mode ON" << endl;
			}else {
				cross_flag = 0;
			cout << "Cross mode OFF" << endl;
			}
		}
		
		//change to link control mode
		if (command_flag&&controller.press(CROSS)) {
			if (!link_flag) {
				link_flag = 1;
				digitalWrite(cross_check,1);
				cout << "Link mode ON" << endl;
			}else {
				link_flag = 0;
				sm.send(12,2,0);
				digitalWrite(cross_check,0);
				cout << "Link mode OFF" << endl;
			}
		}

		//change to arm control mode
		if (command_flag&&controller.press(DOWN)) {
			cross_flag = 0;
			digitalWrite(cross_check,0);
			if(!arm_flag) {
				arm_flag = 1;
				digitalWrite(arm_check,1);
				cout << "Arm mode ON" << endl;
			}else {
				arm_flag = 0;
				digitalWrite(arm_check,0);
				cout << "Arm mode OFF" << endl;
			}
		}
	
		/*//start automated constraction
		if (command_flag&&controller.press(CIRCLE)) {
			auto_constract();
			if (controller.button(L1)) magni_1 = 0.5;
			else magni_1 = 1;
			if (controller.button(R1)) magni_2 = 0.175;
			else magni_2 = 1;
		}*/
		
		//release Naruto bridge
		if (command_flag&&!controller.button(LEFT)&&controller.press(TRIANGLE)) {
			digitalWrite(motor1_IN1,1);
			digitalWrite(motor1_IN2,0);
			//cout << "Motor1 ON" << endl;
		}
		if (command_flag&&controller.button(LEFT)&&controller.press(TRIANGLE)){
			digitalWrite(motor1_IN1,0);
			digitalWrite(motor1_IN2,1);
			//cout << "Motor1 C_ON" << endl;
		}
		if (controller.release(TRIANGLE)) {
			digitalWrite(motor1_IN1,0);
			digitalWrite(motor1_IN2,0);
			//cout << "Motor1 OFF" << endl;
		}

		//release Akashi bridge
		if (command_flag&&!controller.button(LEFT)&&controller.press(SQUARE)) {
			digitalWrite(motor2_IN1,1);
			digitalWrite(motor2_IN2,0);
			//cout << "Motor2 ON" << endl;
		}
		if (command_flag&&controller.button(LEFT)&&controller.press(SQUARE)){
			digitalWrite(motor2_IN1,0);
			digitalWrite(motor2_IN2,1);
			//cout << "Motor2 C_ON" << endl;
		}
		if (controller.release(SQUARE)) {
			digitalWrite(motor2_IN1,0);
			digitalWrite(motor2_IN2,0);
			//cout << "Motor2 OFF" << endl;
		}
		
		//control electromagnet for catching object
		if (!command_flag&&!arm_flag&&controller.press(SQUARE)){
			if (magnet_gate){
				magnet_gate = 0;	
			}else{
				magnet_gate = 1;
			}
			sending_check_magnet = 1;
		}		
		if (sending_check_magnet){
			if (magnet_gate){
				sm.send(13,3,max_pwm,false);
				digitalWrite(magnet_check,1);
				//cout << "Magnet ON" << endl;
			}
			else{
				sm.send(13,3,0,false);
				digitalWrite(magnet_check,0);
				//cout << "Magnet OFF" << endl;
			}
			sending_check_magnet = 0;
		}

		if (controller.press(START))valve_switch = 1;
	
		//control solenoid valve r
		if (!command_flag&&!arm_flag&&controller.press(CIRCLE)){
		switch(valve_switch){
			case(1):{
			pump_gate = 1;
			sending_check_pump = 1;
			valve_switch = 2;
			break;
			}
			case(2):{ 
			start_time_r = millis();
			digitalWrite(solenoid_valve_r,0);
			//cout << "Valve_R ON" << endl;
			digitalWrite(solenoid_valve_l,0);
			sending_check_valve_r = 1;
			valve_switch = 3;
			break;
			}
			case(3):{ 
			start_time_l = millis();
			digitalWrite(solenoid_valve_r,0);
			//cout << "Valve_R ON" << endl;
			digitalWrite(solenoid_valve_l,0);
			//cout << "Valve_L ON" << endl;
			sending_check_valve_l = 1;
			valve_switch = 2;
			break;
			}	
		}
		}
		
		/*//control solenoid valve l
		if (!command_flag&&!arm_flag&&controller.press(SQUARE)){
			start_time_l = millis();
			digitalWrite(solenoid_valve_l,1);
			cout << "Valve_L ON" << endl;
			sending_check_valve_l = 1;
		}
		if (sending_check_valve_l){
			if ((millis()-start_time_l)>5){
				digitalWrite(solenoid_valve_l,0);
				cout << "Valve_L OFF" << endl;
				sending_check_valve_l = 0;
			}
		}*/

		//control vacuum pump
		if (command_flag&&!arm_flag&&controller.press(RIGHT)){
			if (pump_gate){
				pump_gate = 0;	
			}else{
				pump_gate = 1;
			}
			sending_check_pump = 1;
		}		
		if (sending_check_pump){
			if (pump_gate){
				sm.send(10,3,max_pwm,false);
				digitalWrite(pump_check,1);
				//cout << "Pump ON" << endl;
			}
			else{
				sm.send(10,3,0,false);
				digitalWrite(pump_check,0);
				//cout << "Pump OFF" << endl;
			}
			sending_check_pump = 0;
		}
		
		/*if (sending_check_pump_0&&(valve_gate_r||valve_gate_l)){
			sm.send(21,3,max_pwm,false);
			cout << "Pump ON" << endl;		
			sending_check_pump_0 = 0;
			sending_check_pump_1 = 1;
		}
		if (sending_check_pump_1&&(!valve_gate_r&&!valve_gate_l)) {
			sm.send(21,3,0,false);
			cout << "Pump OFF" << endl;
			sending_check_pump_1 = 0;
			sending_check_pump_0 = 1;
		}*/

		if (!command_flag&&!cross_flag&&!arm_flag){
		if (!controller.button(LEFT)&&!digitalRead(over_limit)&&controller.press(UP)) {
			sending_check_up = 1;
			over_limit_flag = 0;
		}
		if (sending_check_up){
			if (digitalRead(hight_200_detect)){
				//catcher of above up
				sm.send(9,3,max_pwm*magnification-20);
				//cout << "Catcher of above UP" <<endl;
			}else{
				sm.send(9,3,0);
			}
			//catcher of below up
			sm.send(21,3,max_pwm*magnification);
			//cout << "Catcher of below UP" <<endl;
		}
		if (!controller.button(LEFT)&&!digitalRead(under_limit)&&controller.press(DOWN)) {
			sending_check_down = 1;
			under_limit_flag = 0;
		}
		if (sending_check_down){	
			if (digitalRead(hight_200_detect)){
				//catcher of below down
				sm.send(21,3,-max_pwm*magnification);
				//cout << "Catcher of below DOWN" << endl;
			}else {
				sm.send(21,3,0);
			}
			//catcher of above down
			sm.send(9,3,-max_pwm*magnification+40);
			//cout << "Catcher of above DOWN" << endl;
		}
		if (controller.release(UP)||controller.release(DOWN)) {
			sm.send(9,3,0);
			sm.send(21,3,0);
			sending_check_up = 0;
			sending_check_down = 0;
		}
		if (digitalRead(over_limit)){
			if (!over_limit_flag) {
				sm.send(9,3,0);
				sm.send(21,3,0);
				sending_check_up = 0;
				over_limit_flag = 1;
			}
		}
		if (digitalRead(under_limit)){
			if (!under_limit_flag) {
				sm.send(9,3,0);
				sm.send(21,3,0);
				sending_check_down = 0;
				under_limit_flag = 1;
			}
		}

		if (!command_flag&&controller.press(RIGHT)/*&&!digitalRead(hold_a_detect)*/){
			//catcher of above close
			sm.send(19,3,magnification*max_pwm/2,false);
			//cout << "Catcher of above CLOSE" << endl;
			hold_a_detect_flag = 0;
		}
		if (!command_flag&&controller.press(RIGHT)/*&&!digitalRead(hold_b_detect)*/){
			//catcher of below close
			sm.send(12,3,magnification*max_pwm/2,false);
			//cout << "Catcher of below CLOSE" << endl;
			hold_b_detect_flag = 0;
		}
		if (!command_flag&&controller.press(LEFT)&&!digitalRead(open_a_limit)){
			//catcher of below open
			sm.send(19,3,-magnification*max_pwm/2,false);
			//cout << "Catcher of above OPEN" << endl;
			open_a_limit_flag = 0;
		}
		if (!command_flag&&controller.press(LEFT)&&!digitalRead(open_b_limit)){
			//catcher of below open
			sm.send(12,3,-magnification*max_pwm/2,false);
			//cout << "Catcher of below OPEN" << endl;
			open_b_limit_flag = 0; 
		}
		if (controller.release(RIGHT)||controller.release(LEFT)) {
			sm.send(19,3,0);
			sm.send(12,3,0);
		}
		/*if (digitalRead(hold_a_detect)) {
			if (!hold_a_detect_flag) {
				sm.send(19,3,0);
				hold_a_detect_flag = 1;
			}
		}
		if (digitalRead(hold_b_detect)) {
			if (!hold_b_detect_flag) {
				sm.send(12,3,0);
				hold_b_detect_flag = 1;
			}
		}*/
		if (digitalRead(open_a_limit)){
			if (!open_a_limit_flag) {
				sm.send(19,3,0);
				open_a_limit_flag = 1;
			}
		}
		if (digitalRead(open_b_limit)){
			if (!open_b_limit_flag) {
				sm.send(12,3,0);
				open_b_limit_flag = 1;
			}
		}
		}

		//independent control
		if (!command_flag&&arm_flag){
		if (!command_flag&&!digitalRead(over_limit)&&controller.press(UP)) {
			sending_check_up = 1;
			over_limit_flag = 0;
		}
		if (sending_check_up){
			//catcher of above up
			sm.send(9,3,max_pwm*magnification);
			//cout << "Catcher of above UP" << endl;
		}
		if (!command_flag&&!digitalRead(hight_200_detect)&&controller.press(TRIANGLE)) {
			sending_check_uup = 1;
			midover_limit_flag = 0;
		}
		if (sending_check_uup){
			//catcher of below up
			sm.send(21,3,max_pwm*magnification/2);
			//cout << "Catcher of below UP" << endl;
		}
		if (!command_flag&&!digitalRead(under_limit)&&controller.press(CROSS)) {
			sending_check_udown = 1;
			under_limit_flag = 0;
		}
		if (sending_check_udown){	
			//catcher of below down
			sm.send(21,3,-max_pwm*magnification);
			//cout << "Catcher of below DOWN" << endl;
		}
		if (!command_flag&&!digitalRead(hight_200_detect)&&controller.press(DOWN)){
			sending_check_down = 1;
			midunder_limit_flag = 0;
		}
		if (sending_check_down){
			//catcher of above down
			sm.send(9,3,-max_pwm*magnification+10);
			//cout << "Catcher of above DOWN" << endl;
		}
		if (controller.release(UP)||controller.release(DOWN)) {
			sm.send(9,3,0);
			sending_check_up = 0;
			sending_check_down = 0;
		}
		if (controller.release(TRIANGLE)||controller.release(CROSS)) {
			sm.send(21,3,0);
			sending_check_uup = 0;
			sending_check_udown = 0;
		}

		if (digitalRead(over_limit)){
			if (!over_limit_flag) {
				sm.send(9,3,0);
				sending_check_up = 0;
				over_limit_flag = 1;
			}
		}
		if (digitalRead(hight_200_detect)){
			if (!midunder_limit_flag) {
				sm.send(9,3,0);
				sending_check_down = 0;
				midunder_limit_flag = 1;
			}
		}
		if (controller.release(UP)||controller.release(DOWN)) {
			sm.send(21,3,0);
			sending_check_uup = 0;
			sending_check_udown = 0;
		}
		if (digitalRead(hight_200_detect)){
			if (!midover_limit_flag) {
				sm.send(21,3,0);
				sending_check_uup = 0;
				midover_limit_flag = 1;
			}
		}
		if (digitalRead(under_limit)){
			if (!under_limit_flag) {
				sm.send(21,3,0);
				sending_check_udown = 0;
				under_limit_flag = 1;
			}
		}

		if (!command_flag&&controller.press(RIGHT)/*&&!digitalRead(hold_a_detect)*/){
			//catcher of above close
			sm.send(19,3,magnification*max_pwm/2,false);
			//cout << "Catcher of above CLOSE" << endl;
			hold_a_detect_flag = 0;
		}
		if (!command_flag&&controller.press(SQUARE)/*&&!digitalRead(hold_b_detect)*/){
			//catcher of below close
			sm.send(12,3,magnification*max_pwm/2,false);
			//cout << "Catcher of below CLOSE" << endl;
			hold_b_detect_flag = 0;
		}
		if (!command_flag&&controller.press(LEFT)&&!digitalRead(open_a_limit)){
			//catcher of above open
			sm.send(19,3,-magnification*max_pwm/2,false);
			//cout << "Catcher of above OPEN" << endl;
			open_a_limit_flag = 0;
		}
		if (!command_flag&&controller.press(CIRCLE)&&!digitalRead(open_b_limit)){
			//catcher of below open
			sm.send(12,3,-magnification*max_pwm/2,false);
			//cout << "Catcher of below OPEN" << endl;
			open_b_limit_flag = 0; 
		}
		if (controller.release(RIGHT)||controller.release(LEFT)) {
			sm.send(19,3,0);
		}
		if (controller.release(SQUARE)||controller.release(CIRCLE)) {
			sm.send(12,3,0);
		}
		/*if (digitalRead(hold_a_detect)) {
			if (!hold_a_detect_flag) {
				sm.send(19,3,0);
				hold_a_detect_flag = 1;
			}
		}
		if (digitalRead(hold_b_detect)) {
			if (!hold_b_detect_flag) {
				sm.send(12,3,0);
				hold_b_detect_flag = 1;
			}
		}*/
		if (digitalRead(open_a_limit)){
			if (!open_a_limit_flag) {
				sm.send(19,3,0);
				open_a_limit_flag = 1;
			}
		}
		if (digitalRead(open_b_limit)){
			if (!open_b_limit_flag) {
				sm.send(12,3,0);
				open_b_limit_flag = 1;
			}
		}
		}

		//control roger arm
		if(!command_flag&&!arm_flag&&controller.press(TRIANGLE)){
			sm.send(6,3,-magnification*max_pwm);
			//cout << "Roger arm UP" << endl;
		}
		if(!command_flag&&!arm_flag&&controller.press(CROSS)){
			sm.send(6,3,magnification*max_pwm);
			//cout << "Roger arm DOWN" << endl;
		}
		if(controller.release(TRIANGLE)||controller.release(CROSS)) sm.send(6,3,0);

		if (sending_check_valve_r){
			if ((millis()-start_time_r)>1){
				digitalWrite(solenoid_valve_r,1);
				//cout << "Valve_R OFF" << endl;
				digitalWrite(solenoid_valve_l,0);
				//cout << "Valve_L OFF" << endl;
				sending_check_valve_r = 0;
			}
		}
		if (sending_check_valve_l){
			if ((millis()-start_time_l)>1){
				digitalWrite(solenoid_valve_r,0);
				//cout << "Valve_R OFF" << endl;
				digitalWrite(solenoid_valve_l,1);
				//cout << "Valve_L OFF" << endl;
				sending_check_valve_l = 0;
			}
		}
		//control ashimawari

		if(!cross_flag){
		//control left side motor
		left_y = controller.stick(LEFT_Y);
		left_x = controller.stick(LEFT_X);

		left_y = -145*left_y/128;
		left_x = 145*left_x/128;

		left_front = (left_x*sqrt(2.)/2)+(left_y*sqrt(2.)/2);
		left_rear = (left_x*sqrt(2.)/2)-(left_y*sqrt(2.)/2);

		if(left_front>max_pwm)left_front=max_pwm;
		if(left_front<-max_pwm)left_front=-max_pwm;
		if(left_rear>max_pwm)left_rear=max_pwm;
		if(left_rear<-max_pwm)left_rear=-max_pwm;
		//printf("%lf\n",magnification*left_front);
	
		//printf("%lf\n",left_y);

		//control right side motor
		right_y = controller.stick(RIGHT_Y);
		right_x = controller.stick(RIGHT_X);

		right_y = -145*right_y/128;
		right_x = 145*right_x/128;

		right_rear = (right_x*sqrt(2)/2)+(right_y*sqrt(2)/2);
		right_front = (right_x*sqrt(2)/2)-(right_y*sqrt(2)/2);

		if(right_rear>max_pwm)right_rear=max_pwm;
		if(right_rear<-max_pwm)right_rear=-max_pwm;
		if(right_front>max_pwm)right_front=max_pwm;
		if(right_front<-max_pwm)right_front=-max_pwm;
	
		}
		else{
			if(controller.press(UP)){
				cout << "UP" << endl;
				left_front = max_pwm;
				left_rear = -max_pwm;
				right_front = -max_pwm;
				right_rear = max_pwm;
			}
			if(controller.press(DOWN)){
				cout << "DOWN" << endl;
				left_front = -max_pwm;
				left_rear = max_pwm;
				right_front = max_pwm;
				right_rear = -max_pwm;
			}
			if(controller.press(RIGHT)){
				cout << "RIGHT" << endl;
				left_front = max_pwm;
				left_rear = max_pwm;
				right_front = max_pwm;
				right_rear = max_pwm;
			}
			if(controller.press(LEFT)){
				cout << "LEFT" << endl;
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

		if(dual_flag){
			sm.send(9,2,-left_front*magnification,false);
			sm.send(21,2,left_rear*magnification,false);
			sm.send(19,2,-left_rear*magnification,false);
			sm.send(6,2,left_front*magnification,false);
			/*printf("left_front:%lf\t",left_front);
			printf("left_rear:%lf\n",left_rear);
			printf("left_rear:%lf\t",left_rear);
			printf("left_y:%lf\n\n",left_front);*/

	}else{
			sm.send(9,2,-left_front*magnification,false);
			sm.send(21,2,left_rear*magnification,false);
			sm.send(19,2,-right_front*magnification,false);
			sm.send(6,2,right_rear*magnification,false);
	}
		
		if(link_flag){
			//cout << (1.3*right_x+1.3*left_x)/2 << endl;
			sm.send(12,2,(1.3*right_x+1.3*left_x)/2);
		}

	//digitalWrite(arm_check,0);
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

void auto_constract(void){

     char accomplishment = 0;
     bool sgn[20] = {0};
     unsigned int tmg[10] = {0};
     bool timer_flag = 0;
     int swct[5] = {0};
     //bool prev_sw[2] = {0};
     //bool pres_sw[2] = {0};

     //prev_sw[0] = digitalRead(hight_a_detect);
     //prev_sw[1] = digitalRead(hight_b_detect);
     bool check_sw[2] = {0};
     bool detect_flag[2] = {0};

     int sampling[2] = {1};

    cout << "Start constracting…"  << endl;

     /*bool HoldFlag = false;    //箱を挟んだかどうか
     bool TwoFlag = false;     //200mmの間隔にする準備ができているか*/
     UPDATELOOP(controller,!controller.button(R1)||!controller.button(L1)||!controller.press(CIRCLE)){

     //終了判定
     if(accomplishment > 5) break;

     //強制的に進めるボタン
     if(controller.press(UP)) {
	accomplishment ++;
	sm.send(19,3,0);
	sm.send(12,3,0);
	sm.send(9,3,0);
	sm.send(21,3,0);
     }
     if(controller.press(DOWN)) {
	accomplishment --;
	sm.send(19,3,0);
	sm.send(12,3,0);
	sm.send(9,3,0);
	sm.send(21,3,0);
     }

     /*//上限・下限指定
     if(digitalRead(over_limit)){
       sm.send(21, 2, 0);
     }
     if(digitalRead(under_limit)){
       sm.send(21, 3, 0);
     }*/

     if(accomplishment == 0){
        //上のアームで箱を挟む
       if(!digitalRead(hold_a_detect)&&!sgn[0]){
         sm.send(19, 3, 100);
         cout << "上の箱を挟みかけ…" << endl;
         sgn[0] = 1;
	 sgn[1] = 0;
       }
       if(digitalRead(hold_a_detect)&&!sgn[1]){
         sm.send(19, 3, 0);
         cout << "上の箱保持完了" << endl;
         sgn[1] = 1;
	 sgn[0] = 0;
       }
       //下のアームで箱を挟む
       if(!digitalRead(hold_b_detect)&&!sgn[2]){
         sm.send(12, 3, 100);
         cout << "下の箱を挟みかけ…" << endl;
         sgn[2] = 1;
	 sgn[3] = 0;
       }
       if(digitalRead(hold_b_detect)&&!sgn[3]){
         sm.send(12, 3, 0);
         cout << "下の箱保持完了" << endl;
         sgn[3] = 1;
	 sgn[2] = 0;
       }
       if(digitalRead(hold_a_detect)&&digitalRead(hold_b_detect)){
         cout << "1_complete" << endl;
         accomplishment ++;
       }
     }

     //高さ確認用リミットスイッチの状態を確認
    if (!check_sw[0]){
	sampling[0] *= digitalRead(hight_a_detect);
        //cout << digitalRead(hight_a_detect) << endl;
        check_sw[0] = 1;
        tmg[1] = millis();
        swct[2]++;
     }
     if ((millis()-tmg[1])>10){
        check_sw[0] = 0;
     }
     if (swct[2] > 2){
        //cout << sampling[0] << endl;
        if(sampling[0]&&detect_flag[0]){
            swct[0]++;
            cout << "上検知" << swct[0] << endl;
            detect_flag[0] = 0;
         }
         else if (!sampling[0]){
            detect_flag[0] = 1;
        }
        sampling[0] = 1;
        swct[2] = 0;
    }

     if (!check_sw[1]){
	sampling[1] *= digitalRead(hight_b_detect);
	//cout << sampling[1] << endl;
        check_sw[1] = 1;
        tmg[2] = millis();
        swct[3]++;
     }
     if ((millis()-tmg[2])>10){
        check_sw[1] = 0;
     }
     if (swct[3] > 2){
        //cout << sampling[1] << endl;
	if(sampling[1]&&detect_flag[1]){
           swct[1]++;
           cout << "下検知" << swct[1] << endl;
           detect_flag[1] = 0;
         }
	else if (!sampling[1]){
           detect_flag[1] = 1;
        }
        sampling[1] = 1;
        swct[3] = 0;
     }

    /*pres_sw[0] = digitalRead(hight_a_detect);
     if(pres_sw[0] != prev_sw[0]){
       tmg[1] = millis();
       if((tmg[1]-tmg[0] > 50)){
         tmg[0] = tmg[1];
         if(digitalRead(hight_a_detect)){
           swct[0] ++;
           cout << "上検知" << swct[0] << endl;
         }
       }
       prev_sw[0] = pres_sw[0];
     }
     pres_sw[1] = digitalRead(hight_b_detect);
     if(pres_sw[1] != prev_sw[1]){
       tmg[3] = millis();
       if((tmg[3]-tmg[2] > 50)){
         tmg[2] = tmg[3];
         if(digitalRead(hight_b_detect)){
          swct[1] ++;
           cout << "下検知" << swct[1] << endl;
         }
       }
       prev_sw[1] = pres_sw[1];
     }*/

     //箱を持ち上げて箱を回転させる
     if(accomplishment == 1){
       //上、下のアームともに上にあげる
       if(!sgn[4]){
         cout << "上アーム上昇" << endl;
         sm.send(9, 3, 190);
         cout << "下アーム上昇" << endl;
         sm.send(21, 3, 70);
         sgn[4] = 1;
       }
       //行くべき場所まで感知したら一旦停止
       if(!sgn[5]&&swct[0] == 4){
           sm.send(9, 3, 0);
           cout << "上アーム停止" << endl;
           sgn[5] = 1;
       }
       if(!sgn[6]&&swct[1] == 2){
           sm.send(21, 3, 0);
           cout << "下アーム停止" << endl;
           sgn[6] = 1;
	   //sm.send(9,3,0);
	   //sgn[5] = 1;
       }
       if(swct[0] >= 4&&swct[1] >= 2){
         if(!sgn[17]){
           tmg[4] = millis();
	   timer_flag = 1;
           sgn[17] = 1;
         }
         //なんとなく待つ
	 if(timer_flag){
         if(millis()-tmg[4] > 100){
           //位置情報のリセット
           swct[0] = 0;
           swct[1] = 0;
	      timer_flag = 0;
           cout << "2_complete" << endl;
           accomplishment ++;
      }
         }
       }
     }

     if(accomplishment == 2){
       //上、下のアーム下降
       if(!sgn[7]){
         sm.send(9, 3, -100);
         cout << "上アーム下降" << endl;
         sm.send(21, 3, -100);
         cout << "下アーム下降" << endl;
         sgn[7] = 1;
       }
       //行くべき場所まで感知したら一旦停止
       if(!sgn[8]&&swct[0] == 1){
           sm.send(9, 3, 0);
           cout << "上アーム停止" << endl;
           sgn[8] = 1;
       }
       if(!sgn[9]&&swct[1] == 1){
           sm.send(21, 3, 0);
           cout << "下アーム停止" << endl;
           sgn[9] = 1;
       }
       if(swct[0] >= 1&&swct[1] >= 1){
         cout << "3_complete" << endl;
         accomplishment ++;
       }
     }

     if(accomplishment == 3){
         //上下のアームで挟んでいる箱を離す
         if(!sgn[10]){
           cout << "上の保持を解除中です" << endl;
           sm.send(19, 3, -100);
           //cout << "下の保持を解除中です" << endl;
           //sm.send(12, 3, -100);
           sgn[10] = 1;
         }
         if(!sgn[18]){
           tmg[5] = millis();
	      timer_flag = 1;
           sgn[18] = 1;
         }
	 if(timer_flag){
         if(/*digitalRead(open_a_limit)*/(millis()-tmg[5]) > 200){
           cout << "上の保持の解除完了" << endl;
           sm.send(19, 3, 0);
           //cout << "下の保持の解除完了" << endl;
           //sm.send(12, 3, 0);
           cout << "4_complete" << endl;
           accomplishment ++;
	      }
         }
     }

     if(accomplishment == 4){
       if(!sgn[12]){
          sm.send(9, 3, -80);
          cout << "上アームを下アームより200mmの間隔まで調整中です" << endl;
          sgn[12] = 1;
       }
       if(digitalRead(hight_200_detect)){
           sm.send(9, 3, 0);
           cout << "200mmの間隔調整完了" << endl;
           cout << "5_complete" << endl;
           accomplishment ++;
       }
     }

     if(accomplishment == 5){
        //上のアームで箱を挟み直す
       if(!digitalRead(hold_a_detect)&&!sgn[13]){
         sm.send(19, 3, 100);
         cout << "上の箱を挟みかけ…" << endl;
         sgn[13] = 1;
         sgn[14] = 0;
       }
       if(digitalRead(hold_a_detect)&&!sgn[14]){
         sm.send(19, 3, 0);
         cout << "上の箱保持完了" << endl;
         sgn[14] = 1;
         sgn[13] = 0;
       }
       //下のアームで箱を挟み直す
       if(!digitalRead(hold_b_detect)&&!sgn[15]){
         sm.send(21, 3, 100);
         cout << "下の箱を挟みかけ…" << endl;
         sgn[15] = 1;
         sgn[16] = 0;
       }
       if(digitalRead(hold_b_detect)&&!sgn[16]){
         sm.send(21, 3, 0);
         cout << "下の箱保持完了" << endl;
         sgn[16] = 1;
         sgn[15] = 0;
       }
       if(digitalRead(hold_a_detect)&&digitalRead(hold_b_detect)){
         cout << "6_complete" << endl;
         accomplishment ++;
       }
     }
   }
    cout << "Constructed" << endl;
	sm.send(19,3,0);
	sm.send(12,3,0);
	sm.send(9,3,0);
	sm.send(21,3,0);
}
