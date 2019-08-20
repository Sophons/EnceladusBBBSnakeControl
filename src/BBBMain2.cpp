#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "TrajPlanner.h"
#include <iostream>
#include <cstring>
#include <math.h>
#include <vector>
#include <memory>



#include "motor-control/MotorControl.h"

#define PI 3.1415
#define maxVelocity 0.2
#define maxEffort 5.0
#define NUM_MOTORS 3

using namespace std;

//Initialize some global memories
double pitchCurrent = 0.0;
double yawCurrent = 0.0;
double period = 0.01;
double rate = 100.0;
bool newCommandSent = false;
int robotStateNum = 1;

const unsigned int enc_addr[] = { 0x40, 0x41, 0x42 }; // Encoder addresses
const int Kp = 0.08, Ki = 0.0, Kd = 0.0; // PID values


//Initialize some messages
//std_msgs::String RobotState;
sensor_msgs::JointState mostRecentCommand;
sensor_msgs::JointState JointState;

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
	double recordedArch = encoder_list[0]->getAngle();
	double recordedPitch = encoder_list[1]->getAngle();
	double recordedYaw = encoder_list[2]->getAngle();
	
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

		pidctrl_list[0]->setTorque(0.0);
		pidctrl_list[1]->setTorque(0.0);
		pidctrl_list[2]->setTorque(0.0);
	}
	
	string pitchName = JointNameChanger(ns,1);
	string yawName = JointNameChanger(ns,2);
	
	JointState.header.stamp = ros::Time::now();
	JointState.header.frame_id = ns;

	JointState.name[1] = pitchName;
	JointState.name[2] = yawName;
	
	JointState.position[0] = recordedArch;
	JointState.position[1] = recordedPitch;
	JointState.position[2] = recordedYaw;
	
	for (int i = 0; i < 3; i++)
	{
		JointState.velocity[i] = encoder_list[i]->getVelocity();
		//JointState.effort[i] = encoder_list[i]->getTorque();
	}

}

int main(int argc, char** argv){
	ros::init(argc,argv, "BBBMain");
	ros::NodeHandle n;
	ros::Rate loop_rate(rate);
	vector<shared_ptr<Encoder>> encoder_list;
	vector<shared_ptr<PID>> pidctrl_list;

	//Get current namespace
	string ns = ros::this_node::getNamespace();
	if (ns == "//BBB1"){ ns = "/BBB1"; }
	if (ns == "//BBB2"){ ns = "/BBB2"; }
	if (ns == "//BBB3"){ ns = "/BBB3"; }
	if (ns == "//BBB4"){ ns = "/BBB4"; }
	cout << "\n namespace is: " << ns;

	//Default is that we don't have a new message and that we aren't running a traj
	newCommandSent = false;
	bool runningTrajectory = false;
	bool pitchMotionCompleted = true;
	bool yawMotionCompleted = true;

	//Subscribe to /JointCommand for new movement commands
	//Subscribe to /RobotState for movement method setting
	string jointCommandTopic = ns + "/JointCommand";
    string robotStateTopic = ns + "/RobotState";
	ros::Subscriber JointCommand_sub = n.subscribe(jointCommandTopic, 1000,  JointCommand);
	ros::Subscriber RobotState_sub = n.subscribe(robotStateTopic, 1000,  RobotStateSetter);
	
	//Set up publisher for joint data
	ros::Publisher JointState_Pub = n.advertise<sensor_msgs::JointState>("/JointState",1000);
	
	//Initialize TrajectoryPlanner objects for Pitch and Yaw
	TrajectoryPlanner PitchPlanner;
	TrajectoryPlanner YawPlanner;

	
	//Start up encoders and PID controllers
	for (int i = 0; i < NUM_MOTORS; i++) {
		encoder_list[i] = shared_ptr<Encoder>(new Encoder(2, enc_addr[i]));
		pidctrl_list[i] = shared_ptr<PID>(new PID(i+1, Kp, Ki, Kd, encoder_list[i]));
	}
	
	//**********Motor Controller recevies Encoder and PID vectors
	MotorControl* motorControl;
	
	motorControl = new MotorControl(encoder_list,pidctrl_list);
	
	motorControl->start();

	

	//Initialize empty messages so maybe segfault goes away :)))))
	populateMsgs();

	
	//Keep looping our pubs/subs/setters while ros runs!
	while(ros::ok()){
		
		loop_rate.sleep();
		ros::spinOnce();

		//Publish joint state information here!!!
		JointStatePub(ns, pidctrl_list, encoder_list);
		JointState_Pub.publish(JointState);
		
		//If no new command, and trajectory is not in progress, go thru
		//otherwise pause here
		if(!newCommandSent && !runningTrajectory){
			cout <<	"\n Checking newCommand@@runningTraj" << 
					"\n newCommand = " << newCommandSent << " runningTrajectory = " << runningTrajectory;
			continue;
		}
	
		//If new command, clear past traj, set runningTraj false, set pitch/yawCurrent
		if(newCommandSent){ 
			cout << "\n new command! doing newCommand segment";
			////Clear past Traj
				PitchPlanner.clearAll();
				YawPlanner.clearAll();
			////Set runningTraj to false
				runningTrajectory = false;
			////Set current position as pitchCurrent/yawCurrent
				pitchCurrent = encoder_list[1]->getAngle();
				yawCurrent = encoder_list[2]->getAngle();
		}
		
		//Set velocity for archimedes screw here!!!
		pidctrl_list[0]->setTorque(mostRecentCommand.effort[0]);

		cout << "\n Starting switch segment";		
		//Depending on ROBOT_STATE, set points/torques into Casey's controller
		//This switch state only matters for the pitch and yaw
		switch(robotStateNum){
			case 1:{ cout << "\n Starting Case 1";
				double pitchCommand;
				double yawCommand;
				
				//if we get a new command msg, set variables, make new trajectory,
				//and rmb that we are in motion
				if(newCommandSent){
					//Recieve new joint angles from the most recent command
					pitchCommand = mostRecentCommand.position[1];
					yawCommand = mostRecentCommand.position[2];

					//Run TrajectoryPlanner to calculate the trajectories
					PitchPlanner.setTrajectory(pitchCurrent, pitchCommand, maxVelocity, period);
					YawPlanner.setTrajectory(yawCurrent, yawCommand, maxVelocity, period);
					
					//You are about to run a trajectory! Motion is not completed
					pitchMotionCompleted = false;
					yawMotionCompleted = false;
					cout << "\n new command recieved! pitchCommand = " << pitchCommand << " yawCommand = " << yawCommand;
				}
				
				//if pitching or yawing isn't done yet, get/set next point till motion is done
				if (!pitchMotionCompleted || !yawMotionCompleted){ cout << "\n seeing if pitch or yaw is done";
					if(!pitchMotionCompleted){ cout << "\n pitchMotionCompleted = " << pitchMotionCompleted;
						//If pitching in progress, get the next pitch point from traj 
						//and feed it to Casey's controller.
						double nextPitchPoint = PitchPlanner.getNextTrajPoint();
						//CaseyController.setPosition(1,nextPitchPoint);
						pidctrl_list[1]->updatePWM(nextPitchPoint,true);
						cout << " next pitch point is: " << nextPitchPoint;

						//If we hit the last point, we finished the motion
						if(nextPitchPoint == pitchCommand){ cout << "\n ####pitchMotionCompleted####";
							pitchMotionCompleted = true;
						}
					}
					
					if(!yawMotionCompleted){ 
						cout<< "\n yawMotionCompleted = " << yawMotionCompleted;
						//If yawing in progress, get the next yaw point from traj 
						//and feed it to Casey's controller.
						double nextYawPoint = YawPlanner.getNextTrajPoint(); 
						pidctrl_list[2]->updatePWM(nextYawPoint,true);
						cout << " next yaw point is: " << nextYawPoint;
						
						//If we hit the last point, we finished the motion
						if(nextYawPoint == yawCommand){cout << "\n ####yawMotionCompleted####";
							yawMotionCompleted = true;
						}
					}
				cout << "\n did traj segment";
				}
				
				//rmb that we are still running a trajectory motion!
				runningTrajectory = !pitchMotionCompleted || !yawMotionCompleted;
				cout << "\n runningTrajectory = " << runningTrajectory;
				break;
			}
			case 2:{ cout << "\n Starting Case 2";
				//get and set the position commands into Casey's controller
				double pitchPosition = mostRecentCommand.position[1];
				double yawPosition = mostRecentCommand.position[2];

				pidctrl_list[1]->updatePWM(pitchPosition,true);
				pidctrl_list[2]->updatePWM(yawPosition,true);
		
				pitchCurrent = pitchPosition;
				yawCurrent = yawPosition;
				cout << "\n did case 2";
				break;
			}
			case 3:{ cout << "\n Starting Case 3";
				//get and set the velocity commmands into Casey's controller
				double pitchTorque = mostRecentCommand.effort[1];
				double yawTorque = mostRecentCommand.effort[2];
			
				//CaseyController.setEffort(1,pitchTorque);
				//CaseyController.setEffort(2,yawTorque);
				cout << "\n did case 3";
				break;
			}
			default: //No ROBOT_STATE sent! 
				ROS_WARN_STREAM("INVALID ROBOT STATE SENT");
				
		}		
		
		//We are still running the current command! 
		newCommandSent = false;
		cout << "\n reached end of ros::ok loop\n\n\n";
		}
	
	motorControl->stop();
}
