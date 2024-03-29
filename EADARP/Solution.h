#pragma once
#include "Route.h"
#include "Position.h"
#include "Instance.h"
#include <array>

class Solution
{
public:
	/*
	 * Hash map to assign each vehicle to a particular route.
	 */
	std::unordered_map<EAV*, Route> routes;
	std::vector<Request*> rejected, removed;
	std::unordered_map<CStation*, unsigned int> stationVisits;
	std::unordered_map<Request*, EAV*> vehicleOfRequest;
	double total_travel_distance, total_excess_ride_time;
	
    double objectiveValue() const noexcept;


	/**
	 * Add (or update) the route traversed by vehicle of route `r`.
	 *
	 * @param r Route.
	 */
	void addRoute(Route& r);

	/**
	 * Default constructor.
	 */
	Solution();

	/**
	 * Default destructor.
	 */
	~Solution()=default;

	/**
	 * Delete routes without requests accomodated.
	 */
	void deleteEmptyRoutes();

	/*
	* Prints the routes on the screen
	*/
	void Display(int what) const;

	/*
	* Adds the corresponding start and end depots of each vehicle in the routes.
	*/
	void AddDepots();

	/*
	* Calculate the added cost of inserting a request in the solution
	*/
	double getInsertionCost(Request* r, const Position& p);
};
