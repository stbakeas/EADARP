#pragma once
class Node {
public:
	enum class Type {DEPOT,ORIGIN,DESTINATION,CHARGING_STATION};
	int id;
	Type type;
	double x; //Longitude
	double y; //Latitude
	double service_duration;
	int load;
	double earliest;
	double latest;
	double maximum_travel_time;
	double recharge_cost;
	double recharge_rate;


	Node() {};
	Node(int id);
	bool isOrigin();
	bool isDestination();
	bool isChargingStation();
	bool isDepot();
	~Node() {};
};