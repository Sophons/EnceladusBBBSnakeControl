#ifndef BBB_H
#define BBB_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "TrajPlanner.h"
#include <cstring>
#include <math.h>
#include <getopt.h>
#include <iostream>
#include <vector>
#include <memory>
using namespace std;

#include "../../../../motor-control/MotorControl.h"
#include "../../../../motor-control/inih/cpp/INIReader.h"

#define PI 3.1415
#define maxVelocity 0.2
#define maxEffort 5.0
#define NUM_MOTORS 3

const unsigned int enc_addr[] = { 0x40, 0x41 };
const unsigned int adc_ch[] = { 3, 5 };
static int running = 0;

// Config variables
double 	Kp, Ki, Kd;

double 	DutyCycle;

double 	DTInner,
		    DTOutter,
		    MovementFrequency,
		    ZeroAngleM2,
		    ZeroAngleM3,
		    Amplitude;

//Initialize some global memories
double pitchCurrent = 0.0;
double yawCurrent = 0.0;
double period = 0.01;
double rate = 100.0;
bool newCommandSent = false;
int robotStateNum = 1;




//Initialize some messages
//std_msgs::String RobotState;
sensor_msgs::JointState mostRecentCommand;
sensor_msgs::JointState JointState;

/**
 * Handle Ctrl-C interrupt
 */
static void __signal_handler(__attribute__ ((unused)) int dummy) {
  running = 0;
  return;
}

// printed if some invalid argument was given
static void __print_usage(char * argv[]) {
  cerr << "Usage:" << endl;
  cerr << "  " << argv[0] << " [dc] [Kp] [Ki] [Kd]" << endl;
  cerr << "  dc: the duty cycle" << endl;
  cerr << "    -- must be a valid floating point number" << endl;
  cerr << "    -- must be in the interval [0.0, 1.0]" << endl;
  cerr << "  Kp: the P in PID" << endl;
  cerr << "    -- must be a valid floating point number" << endl;
  cerr << "  Ki: the I in PID" << endl;
  cerr << "    -- must be a valid floating point number" << endl;
  cerr << "  Kd: the D in PID" << endl;
  cerr << "    -- must be a valid floating point number" << endl;
}

/**
 * Load values from config.ini
 */
void loadIniConfig(void) {
	INIReader reader("./config.ini");
	if (reader.ParseError() < 0) {
    throw runtime_error(string("cannot load config file."));
  }

  // Load real (floating point double) values
  Kp = reader.GetReal("U-JOINTS", "Kp", -1);
  Ki = reader.GetReal("U-JOINTS", "Ki", -1);
  Kd = reader.GetReal("U-JOINTS", "Kd", -1);

  DutyCycle = reader.GetReal("SCREW", "DutyCycle", -1);

  DTInner = reader.GetReal("CONTROL", "DTInner", -1);
  DTOutter = reader.GetReal("CONTROL", "DTOutter", -1);
  MovementFrequency = reader.GetReal("CONTROL", "MovementFrequency", -1);
  ZeroAngleM2 = reader.GetReal("CONTROL", "ZeroAngleM2", -1);
  ZeroAngleM3 = reader.GetReal("CONTROL", "ZeroAngleM3", -1);
  Amplitude = reader.GetReal("CONTROL", "Amplitude", -1);
}


//Gets called whenever a new joint command is recieved
void JointCommand(const sensor_msgs::JointState CommandMsg)
{//Rmb that we got a new command, and store that msg into mostRecentCommand
	//First do a check to see if joint command is in bounds
	if(fabs(CommandMsg.position[1]) > 90*PI/180 
	|| fabs(CommandMsg.position[2]) > 90*PI/180)
	{ROS_WARN_STREAM("Command at time" << CommandMsg.header.stamp << " has position over +- 90 degrees!");}
	else if (fabs(CommandMsg.velocity[0]) > maxVelocity
			|| fabs(CommandMsg.velocity[1]) > maxVelocity
			|| fabs(CommandMsg.velocity[2]) > maxVelocity)
	{ROS_WARN_STREAM("Command at time" << CommandMsg.header.stamp << " has velocity over " << maxVelocity << "!");}
	else if (fabs(CommandMsg.effort[0]) > maxEffort
			|| fabs(CommandMsg.effort[1]) > maxEffort
			|| fabs(CommandMsg.effort[2]) > maxEffort)
	{ROS_WARN_STREAM("Command at time" << CommandMsg.header.stamp << " has velocity over " << maxEffort << "!");}
	else { //No out of bounds, do the normal code!
	newCommandSent = true;
	mostRecentCommand = CommandMsg;
	cout << 		"\n New command is:" << 
					"\n xPos = " << mostRecentCommand.position[0] <<
					"\n yPos = " << mostRecentCommand.position[1] <<
					"\n zPos = " << mostRecentCommand.position[2] <<
					"\n xEff = " << mostRecentCommand.effort[0]   <<
					"\n yEff = " << mostRecentCommand.effort[1]   <<
					"\n zEff = " << mostRecentCommand.effort[2]		;
	}
}

//Gets called whenever a new robot state is recieved
void RobotStateSetter(const std_msgs::String RobotStateMsg)
{//Sets our new robot state to 1:JGP, 2:JP, 3:TC
	if(RobotStateMsg.data == "JGP")	{ robotStateNum = 1; }
	if(RobotStateMsg.data == "JP")		{ robotStateNum = 2; }
	if(RobotStateMsg.data == "TC")		{ robotStateNum = 3; }
	cout << "\n Now in Robot State " << RobotStateMsg.data << robotStateNum ;
}

string JointNameChanger(string ns, int i){ //Sets the joint names depending on namespace
	if(ns == "/BBB1"){
		switch(i){
			case 1:{ return "joint1p"; }
			case 2:{ return "joint1y"; }
			}
	} else if(ns == "/BBB2"){
		switch(i){
			case 1:{ return "joint2p"; }
			case 2:{ return "joint2y"; }
			}
	} else if(ns == "/BBB3"){
		switch(i){
			case 1:{ return "joint3p"; }
			case 2:{ return "joint3y"; }
			}
	} else if(ns == "/BBB4"){
		switch(i){
			case 1:{ return "joint4p"; }
			case 2:{ return "joint4y"; }
			}
	} else {
		ROS_WARN("Invalid namespace!");
	}
}

void populateMsgs(){ //Used to avoid segfault, initiallizes the messages
	mostRecentCommand.position.resize(3);
	mostRecentCommand.velocity.resize(3);
	mostRecentCommand.effort.resize(3);

	JointState.name.resize(3);
	JointState.position.resize(3);
	JointState.velocity.resize(3);
	JointState.effort.resize(3);
	
	for(int i = 0; i < 3; ++i){
		mostRecentCommand.position[i] = 0.0;
		mostRecentCommand.velocity[i] = 0.0;
		mostRecentCommand.effort[i] = 0.0;
		
		JointState.position[i] = 0.0;
		JointState.velocity[i] = 0.0;
		JointState.effort[i] = 0.0;
		
	}
}

void JointStatePub(string ns, vector<shared_ptr<PID>> pidctrl_list, vector<shared_ptr<Encoder>> encoder_list){ //Fills the JointState message
	//Check if Positions are weird
	double recordedPitch = encoder_list[0]->getAngle();
	double recordedYaw = encoder_list[1]->getAngle();
	
	if(fabs(recordedPitch) > 90*PI/180 
	|| fabs(recordedYaw) > 90*PI/180)
	{ROS_INFO_STREAM("Encoder readings nearing max at time" << ros::Time::now() << " readings are"
				<< 	" pitch = " << recordedPitch
				<< 	" and yaw = " << recordedYaw
					);
	} else if( fabs(recordedPitch) > 110*PI/180
			|| fabs(recordedYaw) > 110*PI/180)
	{ROS_WARN_STREAM(	"Encoder readings over max at time" << ros::Time::now() << " readings are"
			<< 	" pitch = " << recordedPitch
			<< 	" and yaw = " << recordedYaw
			<<	" Setting Motor Torques to 0");

        for(int i = 0; i < pidctrl_list.size(); i++ ){
		    pidctrl_list[i]->setDuty(0.0);
        }
	}
	
	string pitchName = JointNameChanger(ns,1);
	string yawName = JointNameChanger(ns,2);
	
	JointState.header.stamp = ros::Time::now();
	JointState.header.frame_id = ns;

	JointState.name[1] = pitchName;
	JointState.name[2] = yawName;
	
	JointState.position[0] = 0.0;
	JointState.position[1] = recordedPitch;
	JointState.position[2] = recordedYaw;
	
	for (int i = 0; i < 2; i++)
	{
		JointState.velocity[i+1] = encoder_list[i]->getVelocity();
		JointState.effort[i+1] = pidctrl_list[i]->getTorque(adc_ch[i]);
	}

}

#endif //BBB_H