#pragma once
class EAV {
public:
	int id;
	int capacity;
	double total_battery;
	double start_time;
	double end_time;
	double battery_return_percentage;
	double initial_battery;
	double acquisition_cost;
	EAV() {};
	EAV(int id, int capacity, double battery, double start_time, double end_time, double initial_battery, double battery_return_percentage);
	~EAV() {};
	bool operator==(const EAV& a) const;
	bool operator<(const EAV& a) const;
};
