
#pragma once

#include "helpers.h"
#include "Counters.h"
#include "optionsParser.h"

void postProcess(Values result,
				  ISAM2Result isam_result,
				  ISAM2 isam,
				  map<string, vector<int>> A_rows_per_type,
				  Counters &counters,
				  Params &params);