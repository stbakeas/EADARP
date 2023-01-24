#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include "Request.h"
#include "CStation.h"
#include "RandLib.h"
#include "EAV.h"

class Route {
public:
	enum class ChargingPolicy{FULL,MINIMAL,CONSERVATIVE};
	enum class Measure{Time,Distance,Battery};
	bool batteryFeasible;
	bool capacityFeasible;
	bool adaptiveCharging;
	ChargingPolicy policy;
	std::vector<Request*> requests;
	std::unordered_map<Node*, int> node_indices;
	EAV* vehicle;
	std::vector<Node*> path;
	std::vector<double> arrival_times;
	std::vector<double> modified_start_of_service_times;
	std::vector<double> departure_times;
	std::vector<double> waiting_times;
	std::vector<double> start_of_service_times;
	std::vector<double> ride_times;
	std::unordered_map<CStation*, double> charging_times;
	std::vector<double> stay_times;
	std::vector<int> loads;
	std::vector<double> battery;
	std::unordered_map<CStation*, bool> assigned_cs;
	std::vector<std::pair<int, int>> natural_sequences;

	double cost[3];

	Route() {};
	Route(EAV* vehicle);
	bool isEmpty(); //Check if the route contains absolutely no nodes
	bool hasNoRequests(); // Check if the route contains no requests.
	bool isFeasible();
	Request* selectRandomRequest(RandLib randlib);
	bool isInsertionBatteryFeasible(Request* request, int i, int j) const;
	bool isInsertionCapacityFeasible(Request* request, int i, int j) const;
	bool batteryFeasibilityTest(Request* request, int i, int j);
	std::pair<size_t, size_t> getRequestPosition(const Request* r);
	double getEarliestTime(int i); //Get the earliest time that node in position i in the route can be served
	int getLoad(int i); //Get the load at position i
	double getDuration(); //Get the route's total duration
	double getAddedDistance(Node* node, int i,Measure measure=Measure::Distance) const; //For a single node
	double getAddedDistance(Request* request, int i, int j, Measure measure = Measure::Distance) const; //For a request
	double get_forward_time_slack(int i);
	CStation* findBestChargingStationAfter(int i);


	/**
	 * Delete charging stations in which the vehicle does not stay there at all.
	 */
	void deleteRedundantChargingStations();
	void computeChargingTime(int nodePosition);
	void computeLoad(int i);
	void computeArrivalTime(int i);
	void computeModifiedStartOfServiceTime(int i);
	void computeStartOfServiceTime(int i);
	void computeWaitingTime(int i);
	void computeDepartureTime(int i);
	void computeRideTime(int i);
	void computeStayTime(int i);
	void computeBatteryLevel(int i);
	void computeTotalCost();
	void insertNode(Node* node, int index);
	void insertRequest(Request* r, int i, int j);
	void removeNode(int index);
	void removeRequest(Request* request);
	void storeNaturalSequences();
	void updateMetrics(); //Computes all necessary metrics 

	
	/*Input: A sequence of n nodes
	* Output: Schedule that minimizes time window violations
	* Runtime: O(n)
	*/
	void BasicScheduling();

	/*Input: Sequence of n nodes, Existing Schedule
	* Output: Schedule that minimizes ride time violations, while not increasing time window violations
	* Runtime: O(n^2)
	*/
	void LazyScheduling();

	~Route() {};
	
};
