#include "Algorithms.h"
#include "Instance.h"
#include "RandLib.h"
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <vector>
#include <iostream>
#include <cfloat>
#include <algorithm>

namespace algorithms {

	using namespace details;

	namespace details {

		double DistanceBetweenUsers(Request* a, Request* b) {
			double euclidian_destinations =
				sqrt(pow(a->destination->y - b->destination->y, 2) + pow(a->destination->x - b->destination->x, 2));
			double euclidian_origins =
				sqrt(pow(a->origin->y - b->origin->y, 2) + pow(a->origin->x - b->origin->x, 2));
			return euclidian_origins + euclidian_destinations;
		}

		Solution constructInitialSolution(int numberOfIterations) {
			Solution solution;
			RandLib randlib(static_cast<unsigned int>(std::time(nullptr)));
			//Initialize the route of each vehicle with the corresponding depots
			for (EAV* v : inst.vehicles)
			{
				Route r = Route(v);
				r.insertNode(inst.getDepot(v, "start"), 0);
				r.insertNode(inst.getDepot(v, "end"), 1);
				r.updateMetrics();
				solution.addRoute(r);
			}


			Solution best_sol;
			for (int count = 0; count < numberOfIterations; ++count) {
				std::vector<Request*> unassigned = inst.requests;
				Solution current_solution = solution;
				std::vector<Position> batset;
				std::vector<Position> capset;
				while (!unassigned.empty()) {
					Request* request = unassigned.at(randlib.randint(0, unassigned.size() - 1));
					for (EAV* v : inst.vehicles)
					{
						for (size_t i = 0; i < current_solution.routes[v].sequence.size(); ++i)
						{
							for (size_t j = i; j < current_solution.routes[v].sequence.size(); ++j)
							{
								if (current_solution.routes[v].isInsertionBatteryFeasible(request, i, j))
								{
									batset.push_back(Position(v, i, j, nullptr, -1));
									if (current_solution.routes[v].isInsertionCapacityFeasible(request, i, j)) { capset.push_back(Position(v, i, j, nullptr, -1)); }
								}
							}
						}
						//Find all the zero load nodes(except for ending depot)
						if (batset.empty()) {
							std::vector<size_t> zero_load_positions;
							for (size_t i = 0; i < current_solution.routes[v].sequence.size() - 1; i++)
							{
								if (current_solution.routes[v].loads.at(i) == 0) zero_load_positions.push_back(i);
							}
							//Try inserting a charging station after zero-load nodes and then rescan the solution
							for (size_t pos : zero_load_positions) {
								CStation* min_s = current_solution.routes[v].findMinimumDetourReachableStationAfter(pos);
								if (min_s != nullptr) {
									current_solution.routes[v].insertChargingStation(min_s, pos + 1, v->battery);
									current_solution.routes[v].updateMetrics();
									for (size_t i = 0; i < current_solution.routes[v].sequence.size(); ++i)
									{
										for (size_t j = i; j < current_solution.routes[v].sequence.size(); ++j)
										{
											if (current_solution.routes[v].isInsertionBatteryFeasible(request, i, j))
											{
											    batset.push_back(Position(v, i, j, min_s, pos));
												if (current_solution.routes[v].isInsertionCapacityFeasible(request, i, j)) { 
													capset.push_back(Position(v, i, j, min_s, pos)); 
												}
											}
										}
									}
									if (!capset.empty()) break;
									current_solution.routes[v].removeChargingStation(min_s, pos + 1);
									current_solution.routes[v].updateMetrics();
								}

							}

						}

					}
					if (batset.empty() || capset.empty()) {
						current_solution.rejected.push_back(request);
					}
					else {
						Position min_pos = *capset.begin();
						if (min_pos.charging_station != nullptr)
							min_pos.cost += current_solution.routes[min_pos.vehicle].recharge_times[min_pos.charging_station] * min_pos.charging_station->recharge_cost;
						double min_cost = current_solution.routes[min_pos.vehicle].getInsertionCost(request, min_pos.origin_pos, min_pos.dest_pos, true) + min_pos.cost;
						for (std::vector<Position>::iterator st = capset.begin() + 1; st != capset.end(); ++st) {
							if (st->charging_station != nullptr)
								st->cost += current_solution.routes[st->vehicle].recharge_times[st->charging_station] * st->charging_station->recharge_cost;
							double cost = current_solution.routes[st->vehicle].getInsertionCost(request, st->origin_pos, st->dest_pos, true) + st->cost;
							if (cost < min_cost) {
								min_cost = cost;
								min_pos = *st;
							}
						}
						
						current_solution.routes[min_pos.vehicle].insertNode(request->origin, min_pos.origin_pos + 1);
						current_solution.routes[min_pos.vehicle].insertNode(request->destination, min_pos.dest_pos + 2);
						current_solution.routes[min_pos.vehicle].updateMetrics();

						//Remove redundant charging stations from the routes in which the request was not inserted
						for (Position p : capset) {
							if (min_pos.vehicle->id != p.vehicle->id) {
								if (p.charging_station != nullptr) {
									current_solution.routes[p.vehicle].removeChargingStation(p.charging_station, p.cs_pos + 1);
									current_solution.routes[p.vehicle].updateMetrics();
								}
							}
						}

						
							
					}
					unassigned.erase(std::remove(unassigned.begin(), unassigned.end(), request), unassigned.end());
					batset.clear();
					capset.clear();

				}
				if (count == 0 || current_solution.rejected.size() < best_sol.rejected.size()) best_sol = current_solution;
			}

			return best_sol;

		}

		//Battery feasibility constraint check needed when assigning the first m users.
		Solution ParallelInsertionHeuristic() {
			Solution solution;
			RandLib randlib(static_cast<unsigned int>(std::time(nullptr)));
			//Initialize the route of each vehicle with the corresponding depots
			for (EAV* v : inst.vehicles)
			{
				Route r = Route(v);
				r.insertNode(inst.getDepot(v, "start"), 0);
				r.insertNode(inst.getDepot(v, "end"), 1);

				solution.addRoute(r);
			}


			//Sort users based on earliest pick up time
			std::vector<Request*> unassigned = inst.requests;
			std::sort(unassigned.begin(), unassigned.end(),
				[](Request* r1, Request* r2) {return r1->origin->earliest < r2->origin->earliest; });

			std::unordered_map<EAV*, Request*> last_assigned_user;
			std::vector<EAV*> all_vehicles = inst.vehicles;

			//Assign the first *m* requests to *m* random vehicles
			size_t m = inst.vehicles.size();
			if (m > unassigned.size()) m = unassigned.size(); //Handle the case where there are more vehicles.
			//Battery feasibility constraint check needed when assigning the first m users.
			for (size_t i = 0; i < m; ++i)
			{
				Request* r = *unassigned.begin();
				int index_of_random_vehicle = randlib.randint(0, inst.vehicles.size() - 1);
				//Ensure that we have not inserted another request in the route, otherwise try again
				if (solution.routes[inst.vehicles.at(index_of_random_vehicle)].sequence.size() == 2) {
					solution.routes[inst.vehicles.at(index_of_random_vehicle)].insertNode(r->origin, 1);
					solution.routes[inst.vehicles.at(index_of_random_vehicle)].insertNode(r->destination, 2);
					unassigned.erase(unassigned.begin());
					last_assigned_user[inst.vehicles.at(index_of_random_vehicle)] = r;
				}
				else --i;
			}

			while (!unassigned.empty())
			{
				Request* r = *unassigned.begin();
				bool insertionFailed = true;

				//Sort the vehicles based on the distance between their last assigned user and currently selected user
				std::sort(all_vehicles.begin(), all_vehicles.end(),
					[&last_assigned_user, r](EAV* v1, EAV* v2)
					{
						return DistanceBetweenUsers(last_assigned_user[v1], r) < DistanceBetweenUsers(last_assigned_user[v2], r);

					});
				std::vector<Position> feasible;
				for (EAV* v : all_vehicles)
				{
					feasible.clear();
					for (size_t i = 0; i < solution.routes[v].sequence.size(); ++i)
					{
						for (size_t j = i; j < solution.routes[v].sequence.size(); ++j)
						{
							if (solution.routes[v].isInsertionBatteryFeasible(r, i, j) &&
								solution.routes[v].isInsertionCapacityFeasible(r, i, j)) feasible.push_back(Position(v, i, j, nullptr, -1));
						}
					}

					if (!feasible.empty()) {
						//Finding the minimum cost insertion position
						Position min_pos = *feasible.begin();
						double min_cost = solution.routes[min_pos.vehicle].getInsertionCost(r, min_pos.origin_pos, min_pos.dest_pos, false);
						for (std::vector<Position>::iterator st = feasible.begin() + 1; st != feasible.end(); ++st) {
							double cost = solution.routes[st->vehicle].getInsertionCost(r, st->origin_pos, st->dest_pos, false) + st->cost;
							if (cost < min_cost) {
								min_cost = cost;
								min_pos = *st;
							}
						}
						//User Insertion
						solution.routes[min_pos.vehicle].insertNode(r->origin, min_pos.origin_pos + 1);
						solution.routes[min_pos.vehicle].insertNode(r->destination, min_pos.dest_pos + 2);
						last_assigned_user[min_pos.vehicle] = r;
						insertionFailed = false;
						break;
					}

				}
				if (insertionFailed) {
					solution.rejected.push_back(r);
				}
				unassigned.erase(unassigned.begin());



				return solution;

			}
		}
	}
}


