#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include "Request.h"
#include "CStation.h"
#include "RandLib.h"
#include "EAV.h"

double dbl_round(long double number, int precision);

class Route {
public:
	enum class ChargingPolicy{FULL,MINIMAL,CONSERVATIVE};
	bool batteryFeasible;
	bool capacityFeasible;
	bool timeFeasible;
	bool adaptiveCharging;
	ChargingPolicy policy;
	std::vector<Request*> requests;
	std::unordered_map<Node*, int> node_indices;
	EAV* vehicle;
	std::vector<Node*> path;
	std::vector<double> start_of_service_times;
	std::vector<double> waiting_times;
	std::unordered_map<CStation*, double> desiredAmount;
	std::vector<int> loads;
	//Version proposed by Savelsbergh (1995) for the PDPTW
	std::vector<double> FTS;

	std::vector<double> battery;
	std::vector<std::pair<int, int>> natural_sequences;
	double travel_distance, excess_ride_time;
	Route() {};
	Route(EAV* vehicle);
	inline bool isEmpty() { return !path.size(); } //Check if the route contains absolutely no nodes
	inline bool hasNoRequests() { return requests.empty(); } // Check if the route contains no requests.
	inline bool isFeasible() { return batteryFeasible && capacityFeasible && timeFeasible; }
	bool isInsertionCapacityFeasible(Request* request, int i, int j) const;
	bool isInsertionBatteryFeasible(Request* request, int i, int j,bool increaseChargingTime);
	bool isInsertionTimeFeasible(Request* request, int i, int j);
	std::pair<size_t, size_t> getRequestPosition(const Request* r);
	int getLoad(int i); //Get the load at position i
	double getWaitingTime(int index);
	double getRideTime(int index);
	double getDuration(); //Get the route's total duration
	double getAddedDistance(Node* node, int i) const; //For a single node
	double getAddedDistance(Request* request, int i, int j) const; //For a request
	double get_forward_time_slack(int i);
	double get_modified_time_slack(int i);
	CStation* findBestChargingStationAfter(int i, std::unordered_map<CStation*, unsigned int>& stationVisits);


	/**
	 * Delete charging stations in which the vehicle does not stay there at all.
	 */
	void deleteRedundantChargingStations();
	void computeChargingTime(int nodePosition);
	inline void computeLoad(int i);
	void computeStartOfServiceTime(int i);
	void computeBatteryLevel(int i);
	void computeTotalCost(bool debug);
	void insertNode(Node* node, int index);
	void insertRequest(Request* r, int i, int j);
	void removeNode(int index);
	void removeRequest(Request* request);
	void storeNaturalSequences();
	void updateMetrics(bool debug=false); //Computes all necessary metrics 

	
	/*Input: A sequence of n nodes
	* Output: Schedule that minimizes time window violations
	* Runtime: O(n)
	*/
	void BasicScheduling();

	/* Input: Sequence of n nodes, Existing Schedule
	* Output: Schedule that minimizes maximum ride time constraint violations, 
	*         while not increasing time window violations
	* Runtime: O(n^2)
	*/
	void LazyScheduling();

	/*Input: Sequence of n nodes, Existing Schedule
	* Output: Schedule that minimizes excess ride time, while not increasing time window violations
	* Runtime: O(n^2)
	*/
	void RideTimeOrientedScheduling();

	~Route() {};
	
};
