#include "CStation.h"
#include <iostream>

CStation::CStation(int id, double recharge_rate):id(id),recharge_rate(recharge_rate){}

double CStation::getChargedAmount(double stay_time) {
	return std::max(0.0,stay_time * this->recharge_rate);
}

double CStation::getRequiredTime(double current_amount,double desired_amount){
	return std::max(0.0,(desired_amount - current_amount) / this->recharge_rate);
}