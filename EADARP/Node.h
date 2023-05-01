#pragma once
class Node {
public:
	enum class Type {START_DEPOT,END_DEPOT,ORIGIN,DESTINATION,CHARGING_STATION};
	int id;
	Type type;
	double x; //Longitude
	double y; //Latitude
	double service_duration;
	int load;
	double earliest;
	double latest;
	double maximum_travel_time;
	

	Node() {};
	explicit Node(int id);
	bool isOrigin() noexcept;
	bool isDestination() noexcept;
	bool isChargingStation() noexcept;
	bool isStartingDepot() noexcept;
	bool isEndingDepot() noexcept;
	~Node() {};
};