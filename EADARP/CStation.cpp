#include "CStation.h"
#include <math.h>
#include <iostream>

CStation::CStation(int id, double recharge_rate, double recharge_cost) {
	this->id = id;
	this->recharge_rate = recharge_rate;
	this->recharge_cost = recharge_cost;
}

double CStation::getChargedAmount(double stay_time) {
	if (stay_time < 0) return 0;
	return stay_time * this->recharge_rate;
}

double CStation::getRequiredTime(double current_amount,double desired_amount){
	return std::max(0.0,desired_amount-current_amount) / this->recharge_rate;
}