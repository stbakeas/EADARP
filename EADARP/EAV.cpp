#include "EAV.h"

EAV::EAV(int id, int capacity, double battery, double start_time, double end_time, double initial_battery, double minBatteryUponReturn):
	id(id),
	capacity(capacity),
	total_battery(battery),
	start_time(start_time),
	end_time(end_time),
	minBatteryUponReturn(minBatteryUponReturn),
	initial_battery(initial_battery),
	acquisition_cost(battery){}