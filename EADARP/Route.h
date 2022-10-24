#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include "Request.h"
#include "CStation.h"
#include "EAV.h"

class Route {
public:
	std::unordered_map<Node*, int> node_indices;
	EAV* vehicle;
	std::vector<Node*> sequence;
	std::vector<double> arrival_times;
	std::vector<double> departure_times;
	std::vector<double> waiting_times;
	std::vector<double> start_of_service_times;
	std::vector<double> ride_times;
	std::unordered_map<CStation*, double> recharge_times;
	std::vector<double> stay_times;
	std::vector<double> FTS;
	std::vector<int> loads;
	std::vector<double> battery;
	std::vector<std::vector<int>> ml; //ml{i,j} stores the maximum load of the vehicle between nodes i and j.
	std::unordered_map<CStation*, bool> assigned_cs;

	double cost;
	double load_violation;
	double time_window_violation;
	double max_ride_time_violation;
	int returned_battery_violation;

	Route() {};
	Route(EAV* vehicle);
	bool isEmpty(); //Check if the route contains absolutely no nodes
	bool hasNoRequests(); // Check if the route contains no requests.
	bool feasible();
	//We assume the origin will be inserted after position i and the destination after position j in the route.
	bool isInsertionTimeFeasible(Request *request, int i, int j); 
	bool isInsertionBatteryFeasible(Request* request, int i, int j);
	bool isInsertionCapacityFeasible(Request* request, int i, int j);
	double Evaluate();
	double getEarliestTime(int i); //Get the earliest time that node in position i in the route can be served
	int getLoad(int i); //Get the load at position i
	double getDuration(); //Get the route's total duration
	double getAddedDistance(Node* node, int i,bool timeMode=false); //For a single node
	double getAddedDistance(Request* request, int i, int j, bool timeMode = false); //For a request
	double getInsertionCost(Request* request, int i, int j,bool myMode);
	CStation* findMinimumDetourReachableStationAfter(int i);
	void computeForwardTimeSlack(int i); //Get the forward time slack of node at position i
	void computeLoad(int i);
	void computeArrivalTime(int i);
	void computeStartOfServiceTime(int i);
	void computeWaitingTime(int i);
	void computeDepartureTime(int i);
	void computeRideTime(int i);
	void computeStayTime(int i);
	void computeBatteryLevel(int i);
	void computeMaximumLoadBetween(int i, int j);
	void insertNode(Node* node, int index);
	void insertChargingStation(CStation* s, int index, double desired_amount);
	void removeChargingStation(CStation* s,int index);
	void removeNode(int index);
	void removeRequest(Request* request);
	void updateMetrics(); //Computes all necessary metrics 

	~Route() {};
	
};
