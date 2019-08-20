//#pragma once
#include <vector>
#include <math.h>

class TrajectoryPlanner{
private:
	double current_point;
	double end_point;
    std::vector<double> trajectory;
	int current_ind;
	long num_points;
	bool finished;
	bool setting_trajectory;

    double getTrajPoint(long desired_trajNum);

public:
    TrajectoryPlanner();
    ~TrajectoryPlanner();
    bool isFinished();
    double getNextTrajPoint();
    void setTrajectory(double startpoint, double setpoint, double maxVelocity, double period);
    void clearAll();
};
