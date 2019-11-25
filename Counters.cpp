
#include "Counters.h"

using namespace std;
using namespace gtsam;



// ----------------------------------------------------
void Counters::add_factor(string type){
	this->types.push_back(type);
}


// ----------------------------------------------------
void Counters::update_A_rows(const double lag){
	// if there has not being marginalization -> return
	if (this->current_time <= lag){ return; }

	// time at the beggining of the sliding window
	double time_threshold= this->current_time - lag;

	cout<< "Remove rows!\nCurrent time: "<< this->current_time
		<< "\nTime threshold: "<< time_threshold<< endl;


	int num_erased_rows= 0;
	for (map<string, pair_vector>::iterator
  					it_types= this->A_rows.begin(); 
					it_types != this->A_rows.end(); 
					++it_types) {

		// type of the factor
		string type= it_types->first;

		// vector with: (A_rows & time)
		pair_vector &row_time= this->A_rows[type];

		// for each row of this type included
		pair_vector::iterator it_rows= row_time.begin();
		while ( it_rows < row_time.end() ) {
			double msmt_time= it_rows->second;

			// remove factors before the sw
			if (msmt_time < time_threshold) {				
				// cout<< "remove msmt at time: "<< msmt_time 
				// 	<< "\tat row: "<< it_rows->first<< endl;
				row_time.erase(it_rows);
				++num_erased_rows;
			// remove the last odom factor
			} else if (type == "odom" && 
					  msmt_time < time_threshold + this->dt_gps) {
				row_time.erase(it_rows);
				++num_erased_rows;
			} else {
				++it_rows;
			}
		}
	}

	// substract the number of removed factors from the row indexes
	for (map<string, pair_vector>::iterator
  					it_types= this->A_rows.begin(); 
					it_types != this->A_rows.end(); 
					++it_types) {

		string type= it_types->first;

		// vector with: (A_rows & time)
		pair_vector &row_time= this->A_rows[type];

		// for each row of this type included
		for (int i= 0; i < row_time.size(); ++i) {
			row_time[i].first= row_time[i].first - num_erased_rows;
		}
	}

	// keep count of the number of rows in A (this doesn't account for the prior hessina factor)
	this->num_A_rows -= num_erased_rows;
}






