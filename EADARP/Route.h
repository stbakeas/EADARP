#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include "Request.h"
#include "CStation.h"
#include "RandLib.h"
#include "EAV.h"

double dbl_round(long double number, int precision) noexcept;

class Route {
public:
	bool batteryFeasible;
	bool capacityFeasible;
	bool timeFeasible;
	std::vector<Request*> requests;
	std::unordered_map<Node*, int> node_indices;
	EAV* vehicle;
	std::vector<Node*> path;
	std::vector<double> base_schedule,base_waiting_times,late_schedule,late_waiting_times,FTS,battery;
	std::unordered_map<CStation*, double> departureBatteryLevel;
	std::vector<int> loads;
	std::vector<std::pair<int, int>> natural_sequences;
	double travel_distance, excess_ride_time;

	Route();
	explicit Route(EAV* vehicle);

	void PartialCopy(const Route& route);
	inline bool isEmpty() { return !path.size(); } //Check if the route contains absolutely no nodes
	inline bool hasNoRequests() noexcept { return requests.empty(); }  // Check if the route contains no requests.
	inline bool isFeasible() noexcept { return batteryFeasible && capacityFeasible && timeFeasible; }
	bool isInsertionCapacityFeasible(const Request* request, int i, int j) const;
	bool isInsertionBatteryFeasible(const Request* request, int i, int j);
	bool isInsertionTimeFeasible(const Request* request, size_t i, size_t j);
	std::pair<size_t, size_t> getRequestPosition(const Request* r);
	void computeWaitingTime(int index,bool base=true);
	double getRideTime(int index);
	double getAddedDistance(Node* node, int i) const; //For a single node
	double getAddedDistance(const Request* request, int i, int j) const; //For a request
	double getForwardTimeSlack(int i);
	double get_modified_time_slack(int i);
	CStation* findBestChargingStationAfter(int i, std::unordered_map<CStation*, unsigned int>& stationVisits);


	/**
	 * Delete charging stations in which the vehicle does not stay there at all.
	 */
	bool deleteRedundantChargingStations();
	void computeChargingTime(int stationPosition);
	inline void computeLoad(int i);
	void computeStartOfServiceTime(int i,bool base=true);
	void computeBatteryLevel(int i);
	void computeExcessRideTime(bool debug);
	void insertNode(Node* node, int index);
	void insertRequest(Request* r, int i, int j);
	void removeNode(int index);
	void removeRequest(const Request* request);
	void storeNaturalSequences();
	void updateMetrics(bool debug=false); //Computes all necessary metrics 
	
	/*Input: A sequence of n nodes
	* Output: Schedule that minimizes time window violations
	* Runtime: O(n)
	*/
	void BasicScheduling(bool debug);

	/* Input: Sequence of n nodes, Existing Schedule
	* Output: Schedule that minimizes maximum ride time constraint violations, 
	*         while not increasing time window violations
	* Runtime: O(n^2)
	*/
	void LazyScheduling();

	~Route()=default;
	
};
