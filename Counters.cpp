
#include "Counters.h"

using namespace std;
using namespace gtsam;


// ----------------------------------------------------
void Counters::update_A_rows(double lag){
	// if there has not being marginalization -> return
	if (this->current_time <= lag){ return; }

	// time at the beggining of the sliding window
	double time_threshold= this->current_time - lag;

	// cout<< "Remove rows!\nCurrent time: "<< this->current_time
	// 	<< "\nTime threshold: "<< time_threshold<< endl;


	for (map<string, pair_vector>::iterator
  					it_types= this->A_rows.begin(); 
					it_types != this->A_rows.end(); 
					++it_types){

		// type of the factor
		string type= it_types->first;

		// vector with: (A_rows & time)
		pair_vector &row_time= this->A_rows[type];

		// for each row of this type included
		pair_vector::iterator it_rows= row_time.begin();
		while ( it_rows < row_time.end() ) {
			double msmt_time= it_rows->second;

			// remove all factors before the sw
			if (msmt_time < time_threshold){
				// cout<< "remove msmt at time: "<< msmt_time 
				// 	<< "\tat row: "<< it_rows->first<< endl;
				row_time.erase(it_rows);
			// remove the last odom factor
			}else if (type == "odom" && 
					  msmt_time < time_threshold + this->dt_gps){
				row_time.erase(it_rows);
			}else{
				++it_rows;
			}
		}
	}
}
