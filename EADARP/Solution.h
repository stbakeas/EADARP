#pragma once
#include "Route.h"
#include "Position.h"
#include "Instance.h"

class Solution
{
public:
	struct SolutionHash
	{
		std::size_t operator()(const Solution& s) const
		{
			using std::size_t;
			using std::hash;
			using std::string;

			// Compute individual hash values for first,
			// second and third and combine them using XOR
			// and bit shifting:

			return ((hash<float>()(s.objective_value[0])
				^ (hash<float>()(s.objective_value[1]) << 1)) >> 1)
				^ (hash<float>()(s.objective_value[2]) << 1);
		}
	};
	/*
	 * Hash map to assign each vehicle to a particular route.
	 */
	std::unordered_map<EAV*, Route> routes;
	std::vector<Request*> rejected;
	std::vector<std::pair<Request*, EAV*>> removed;
	float weights[static_cast<int>(Instance::Objective::NumberOfObjectives)] = { 0.33,0.33,0.33 };
	double objective_value[static_cast<int>(Instance::Objective::NumberOfObjectives)];
	void SetWeights(std::vector<float> values);

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
	* Function to compare two solutions
	*/
	double AchievementFunction(double rho);

	double WeightedSum();

	double distanceFrom(Solution s, bool objectiveSpace = true);
	/*
	* Calculate the added cost of inserting a request in the solution
	*/
	double getInsertionCost(Request* r, Position p, Instance::Objective objective=Instance::Objective::NumberOfObjectives);
	/*
	* Based on Pareto Dominance Rules
	*/
	bool dominates(Solution s);

	bool operator == (const Solution& s) const;

	bool operator != (const Solution& s) const;

};
