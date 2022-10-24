#include "CStation.h"
#include <math.h>
#include <iostream>

CStation::CStation(int id, double recharge_rate, double recharge_cost) {
	this->id = id;
	this->recharge_rate = recharge_rate;
	this->recharge_cost = recharge_cost;
}

double CStation::getChargedAmount(double stay_time) {
	return stay_time * this->recharge_rate;
}

double CStation::getRequiredTime(double current_amount,double desired_amount){
	if (current_amount > desired_amount) return 0;
	else {
		double diff = desired_amount - current_amount;
		return (diff / this->recharge_rate);
	}
}