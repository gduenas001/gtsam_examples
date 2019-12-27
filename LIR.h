
#pragma once

// -------------------------------------------------------
class H_LIR {
public:
	void set(std::string state,
		double lir){
		if (state == "x") {
			this->x= lir;
		} else if (state == "y") {
			this->y= lir;
		} else if (state == "z") {
			this->z= lir;
		} else if (state == "lateral") {
			this->lateral= lir;
		} else if (state == "longitudinal") {
			this->longitudinal= lir;
		} else if (state == "vertical") {
			this->vertical= lir;
		} else if (state == "roll") {
			this->roll= lir;
		} else if (state == "pitch") {
			this->pitch= lir;
		} else if (state == "yaw") {
			this->yaw= lir;

		} else {
			LOG(WARNING)<< state<< " is not include in H_LIR class";
		}
	};

	double x;
	double y;
	double z;
	double lateral;
	double longitudinal;
	double vertical;
	double roll;
	double pitch;
	double yaw;
};

// -------------------------------------------------------
class LIR {
public:
	/*
	*/
	void set(std::string hypothesis, 
			 H_LIR &h_lir){
		if (hypothesis == "null") {
			this->null= h_lir;
		} else if (hypothesis == "lidar") {
			this->lidar= h_lir;
		} else if (hypothesis == "gps") {
			this->gps= h_lir;
		} else if (hypothesis == "odom") {
			this->odom= h_lir;
		} else {
			LOG(WARNING)<< hypothesis<< " is not include in LIR class";
		}
	};

	H_LIR null;
	H_LIR lidar;
	H_LIR gps;
	H_LIR odom;
};
