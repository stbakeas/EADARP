#pragma once
class CStation {
public:
	int id;
	double recharge_rate;
	double recharge_cost;

	CStation() {};
	CStation(int id, double recharge_rate, double recharge_cost);
	~CStation() {};
	double getChargedAmount(double stay_time);
	double getRequiredTime(double current_amount, double desired_amount);
	double cancer(double getit);
};