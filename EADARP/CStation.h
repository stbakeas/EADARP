#pragma once
class CStation {
public:
	int id;
	double recharge_rate;
	CStation() {};
	CStation(int id, double recharge_rate);
	~CStation()=default;
	double getChargedAmount(double stay_time) const;
	double getRequiredTime(double current_amount, double desired_amount) const;
};