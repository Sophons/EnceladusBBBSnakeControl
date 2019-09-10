#include "BBB.h"

int main(int argc, char** argv){
	ros::init(argc,argv, "BBB");
	ros::NodeHandle n;
	ros::Rate loop_rate(rate);
	
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

	//Initialize empty messages so maybe segfault goes away :)))))
	populateMsgs();


	// Set up encoders and motor PID control
	I2CBus* theBus = new I2CBus(2);
	MotorControl* mc = new MotorControl(DTInner);

	PID* screw = new PID(1);

	for (int i = 0; i < 2; i++) {
		mc->encoder_list.push_back(shared_ptr<Encoder>(new Encoder(theBus, DTInner, enc_addr[i])));
		mc->pidctrl_list.push_back(shared_ptr<PID>(new PID(i+2, DTInner, Kp, Ki, Kd, mc->encoder_list[i])));
	}

	// Establish zero position
	mc->encoder_list[0]->setZeroPosition(ZeroAngleM2);
	mc->encoder_list[1]->setZeroPosition(ZeroAngleM3);

	// Start the motors
	
	mc->start();



	//Keep looping our pubs/subs/setters while ros runs!
	while(ros::ok()){
		
		loop_rate.sleep();
		ros::spinOnce();

		//Publish joint state information here!!!
		JointStatePub(ns, mc->pidctrl_list, mc->encoder_list);
		JointState_Pub.publish(JointState);
		
		//If no new command, and trajectory is not in progress, go thru otherwise pause here
		if(!newCommandSent && !runningTrajectory){
			cout <<	"\n Checking newCommand@@runningTraj" << 
					"\n newCommand = " << newCommandSent << " runningTrajectory = " << runningTrajectory;
			continue;
		}
	
		//If new command, clear past traj, set runningTraj false, set pitch/yawCurrent
		if(newCommandSent){ 
			cout << "\n new command! doing newCommand segment";
			//Clear past Traj
				PitchPlanner.clearAll();
				YawPlanner.clearAll();
			//Set runningTraj to false
				runningTrajectory = false;
			//Set current position as pitchCurrent/yawCurrent
				pitchCurrent = mc->encoder_list[1]->getAngle();
				yawCurrent = mc->encoder_list[2]->getAngle();
		}
		
		//Set dutyCycle for archimedes screw here!!!
		screw->setDuty(mostRecentCommand.velocity[0]);
		cout << "setting duty cycle of screw to " << DutyCycle << endl;


		cout << "\n Starting switch segment";		
		//Depending on ROBOT_STATE, set points/torques into Casey's controller
		//This switch state only matters for the pitch and yaw
		switch(robotStateNum){
			case 1: {cout << "\n Starting Case 1 JointGoalPos";
				double pitchCommand, yawCommand;

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
						mc->pidctrl_list[0]->setAngle(nextPitchPoint);
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
						mc->pidctrl_list[1]->setAngle(nextYawPoint);
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
			case 2: {cout << "\n Starting Case 2 JointPos";
				//get and set the position commands into Casey's controller
				double pitchPosition = mostRecentCommand.position[1];
				double yawPosition = mostRecentCommand.position[2];

				mc->pidctrl_list[0]->setAngle(pitchPosition);
				mc->pidctrl_list[1]->setAngle(yawPosition);
		
				pitchCurrent = pitchPosition;
				yawCurrent = yawPosition;
				cout << "\n did case 2";
				break;
			}
			case 3: {cout << "\n Starting Case 3 TorqueControl";
				//get and set the velocity commmands into Casey's controller
				double pitchTorque = mostRecentCommand.effort[1];
				double yawTorque = mostRecentCommand.effort[2];
			
				//CaseyController.setEffort(1,pitchTorque); DONT HAVE THIS YET
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


	screw->setDuty(0);
  	mc->stop();

  	delete screw;
 	delete mc;
  	delete theBus;
}
