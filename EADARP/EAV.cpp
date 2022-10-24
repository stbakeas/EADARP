#include "EAV.h"
EAV::EAV(int id, int capacity, double battery, double start_time, double end_time, double battery_return_percentage) {
	this->id = id;
	this->capacity = capacity;
	this->battery = battery;
	this->start_time = start_time;
	this->end_time = end_time;
	this->battery_return_percentage=battery_return_percentage;
}

bool EAV::operator==(const EAV& a) {
	return this->id == a.id;
}