#pragma once
#include "Route.h"

class Solution
{
public:
	/*
	 * Hash map to assign each vehicle to a particular route.
	 */
	std::unordered_map<EAV*, Route> routes;
	std::vector<Request*> rejected;

	/*
	 * Solution's total cost.
	 */
	double total_cost;

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
	 * Check if solution is feasible.
	 *
	 * @return `true` if feasible.
	 */
	bool feasible();

	/**
	 * Delete routes without requests accomodated.
	 */
	void deleteEmptyRoutes();
};
