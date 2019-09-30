#include <map>
#include <string>

class Time{
public:
	Time(){
		// this->total_sim_time= 
		this->accum["gps"]= 0;
		this->accum["lidar"]= 0; // not used yet

	}

	~Time;
// 
	// double total_sim_time;
	double current;
	std::map<string, double> accum;

};
