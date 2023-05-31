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
	bool isOrigin() const noexcept;
	bool isDestination() const noexcept;
	bool isChargingStation() const noexcept;
	bool isStartingDepot() const noexcept;
	bool isEndingDepot() const noexcept;
	~Node()=default;
};