#include "EAV.h"

EAV::EAV(int id, int capacity, double battery, double start_time, double end_time, double initial_battery, double battery_return_percentage) {
	this->id = id;
	this->capacity = capacity;
	this->total_battery = battery;
	this->start_time = start_time;
	this->end_time = end_time;
	this->battery_return_percentage = battery_return_percentage;
	this->initial_battery = initial_battery;
	this->acquisition_cost = this->total_battery;
}

bool EAV::operator==(const EAV& a) const {
	return this->id == a.id;
}

bool EAV::operator<(const EAV& a) const {
	return this->id < a.id;
};