#include "TrajPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(){
    clearAll();
}

TrajectoryPlanner::~TrajectoryPlanner(){
    clearAll();
}

void TrajectoryPlanner::setTrajectory(double startpoint, double setpoint, double maxVelocity, double period){
    setting_trajectory = true;
    current_point = startpoint;
    end_point = setpoint;
    
//    num_points = number_of_points;
    double dist = setpoint - startpoint;
	num_points = abs(int( // 1.5*distanceBetweenPoints/(maxVelocity*period)
					ceil( 
							1.5*(dist/(maxVelocity*period))
						)
					));
	
    //create a smooth function (cubic spline):
    //y = -2x^3 + 3x^2
    trajectory.resize(num_points); // Resize trajectory array to fit the amount of num_points
    double ii;
    for(int i = 1; i <= num_points; i++){
        ii = ((double) i)/((double) num_points);
        trajectory[i-1] = dist*(-2.0*ii*ii*ii + 3*ii*ii) + startpoint;
    }
    current_ind = 0;
    finished = false;
    setting_trajectory = false;
}



void TrajectoryPlanner::clearAll()
{
    setting_trajectory = true;
    trajectory.clear();
    current_point = 0.0;
    end_point = 0.0;
    num_points = 1;
    current_ind = 0;
    for(int i = 0; i < num_points; i++){
        trajectory.push_back(1);
    }
    finished = true;
}

double TrajectoryPlanner::getTrajPoint(long desired_trajNum){
    if(desired_trajNum > num_points-1){
        desired_trajNum = num_points-1;
        finished = true;
    }
    current_point = trajectory[desired_trajNum];
    current_ind = desired_trajNum;
    return current_point;
}

double TrajectoryPlanner::getNextTrajPoint(){
    while(setting_trajectory == true){
        //std::cout << "Waiting to set trajectory\n" << #define:endl;
    }
    current_ind++;
    return getTrajPoint(current_ind);
}

bool TrajectoryPlanner::isFinished()
{
    return finished;
}
