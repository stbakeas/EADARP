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
	std::vector<Request*> rejected;
	std::vector<std::pair<Request*, EAV*>> removed;
	double total_travel_distance, total_excess_ride_time;

    double objectiveValue() const;


	/**
	 * Add (or update) the route traversed by vehicle of route `r`.
	 *
	 * @param r Route.
	 */
	void addRoute(Route r);

	/**
	 * Default constructor.
	 */
	Solution();

	/**
	 * Default destructor.
	 */
	~Solution() {};

	/**
	 * Delete routes without requests accomodated.
	 */
	void deleteEmptyRoutes();

	/*
	* Used in Su et. al. (2021) for the EADARP
	*/
	void FixHardConstraints();

	/*
	* Prints the routes on the screen
	*/
	void Display(int what);

	/*
	* Adds the corresponding start and end depots of each vehicle in the routes.
	*/
	void AddDepots();

	/*
	* Calculate the added cost of inserting a request in the solution
	*/
	double getInsertionCost(Request* r, Position p);
};
