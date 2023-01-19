#include "Algorithms.h"
#include "Instance.h"
#include "RandLib.h"
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <vector>
#include <iostream>
#include <cfloat>
#include <functional>
#include <chrono>
#include <unordered_set>
#include <ctime>


namespace algorithms {

    #define getName(VariableName) # VariableName

	using namespace details;

	typedef std::pair<Request*, EAV*> Assignment; 

	bool SimulatedAnnealingAcceptanceCriterion(Solution candidate, Solution incumbent, int current_temperature) {
		std::random_device rd;
		RandLib randlib(rd());
		double diff = incumbent.AchievementFunction(0.3) - candidate.AchievementFunction(0.3);
		if (diff == 0) return false;
		double acceptance_probability = pow(2.71828,diff / current_temperature);
		double p = randlib.unifrnd(0.0, 1.0);
		if (acceptance_probability > p) return true;
		else return false;
	}

	Run MDIG(std::vector<Solution> initPop, int max_iterations, int max_seconds)
	{
		std::vector<Move> intraRouteSequence = { exchangeOrigin,exchangeDestination,exchangeConsecutive,moveStation };
		std::random_device rd;
		RandLib randlib(rd());
		Run run;
		for (Solution sol : initPop) run.archive.insert(sol);
		auto start = std::chrono::steady_clock::now();
		for (int iter = 0; iter < max_iterations; iter++)
		{
			Solution s = run.NaturalSelection();
			std::cout << "Solution selected,";
			for (auto objective : inst.objectives) {

				if (objective != Instance::Objective::System) {
					Solution s_1 = Destroy(s, 0.1, 10, objective);
					run.archive.insert(s_1);
					std::cout << "Destroyed";
					Solution s_2 = Repair(s_1, objective);
					run.archive.insert(s_2);
					std::cout << "Repaired,";
				}
			}
			run.updateArchive();
			std::cout << iter << std::endl;
			auto finish = std::chrono::steady_clock::now();
			double elapsed = std::chrono::duration_cast<std::chrono::seconds>(finish - start).count();
			run.elapsed_seconds = elapsed;
			if (elapsed > max_seconds) break;
		}
		return run;
	}

	Run IteratedGreedy(Solution initial,int max_iterations,int max_seconds)
	{
		std::vector<Move> intraRouteSequence = {exchangeOrigin,exchangeDestination,exchangeConsecutive,moveStation};
		int T_init = 20;
		int statistics[3] = { 0,0,0 };
		double cooling_factor = 0.97;
		std::random_device rd;
		RandLib randlib(rd());
		Run run;
		auto start = std::chrono::steady_clock::now();
		run.init = initial;
		std::cout << "Running ILS..." << std::endl;
		Solution incumbent = run.best=run.init;
		int iter = 0;
		while (iter<max_iterations) {
			Solution s =Destroy(incumbent, 0.1, 7,Instance::Objective::NumberOfObjectives);
			s = Repair(s, Instance::Objective::NumberOfObjectives);
			double before = s.AchievementFunction(0.3);
			for (Move move : intraRouteSequence) s = move(s, NeighborChoice::BEST);
			double after = s.AchievementFunction(0.3);
			if (after < before) statistics[2]++;
			if (s.AchievementFunction(0.3) < incumbent.AchievementFunction(0.3)) {
				incumbent = s;
				if (s.AchievementFunction(0.3) < run.best.AchievementFunction(0.3)) {
					run.best = s;
					statistics[0]++;
					iter = 0;
				}
			}
			else if (SimulatedAnnealingAcceptanceCriterion(s, incumbent, T_init)) {
				incumbent = s;
			}
			iter++;
			std::cout << iter << std::endl;
			if (iter > statistics[1]) statistics[1] = iter;
			auto finish = std::chrono::steady_clock::now();
			double elapsed = std::chrono::duration_cast<std::chrono::seconds>(finish - start).count();
			run.elapsed_seconds = elapsed;
			if (elapsed > max_seconds) break;
			T_init *= cooling_factor;
		}
		std::cout << "\n";
		std::cout << "Times new best solution was found: " << statistics[0] << "\n";
		std::cout << "Most iterations without improvement: " << statistics[1] << "\n";
		std::cout << "Times intra-route local search brought improvement: " << statistics[2] << "\n";
		run.init.deleteEmptyRoutes();
		run.best.deleteEmptyRoutes();
		return run;
	}

	Run DeterministicAnnealing(Solution initial, int Nb_iter, int T_max,int T_red, int n_imp)
	{
		Run run;
		std::vector<Move> neighborhoods = {exchangeOrigin,exchangeDestination,exchangeConsecutive,two_opt,swapRequest,relocate};
		std::random_device rd;
		RandLib randlib(rd());
		int threshold = T_max;
		std::cout << threshold << std::endl;
		int i_imp = 0;
		run.init = initial;
		run.best = run.init;
		Solution incumbent = run.best;
		auto start = std::chrono::steady_clock::now();
		for (int iter = 0; iter < Nb_iter; iter++) {
			i_imp++;
			Solution candidate;
			for (Move move : neighborhoods) {
				candidate = move(incumbent, NeighborChoice::RANDOM);
				candidate.FixHardConstraints();
				if (candidate.AchievementFunction(0.3) < incumbent.AchievementFunction(0.3) + threshold)
					incumbent = candidate;
			}
			incumbent = Repair(incumbent,Instance::Objective::NumberOfObjectives);
			if (incumbent.AchievementFunction(0.3) < run.best.AchievementFunction(0.3)) {
				run.best = incumbent;
				i_imp = 0;
			}
			if (i_imp > 0) {
				threshold -= T_max / T_red;
				if (threshold < 0) {
					threshold = randlib.rand() * T_max;
					if (i_imp > n_imp) {
						incumbent = run.best;
						i_imp = 0;
					}
				}
			}
			auto finish = std::chrono::steady_clock::now();
			double elapsed = std::chrono::duration_cast<std::chrono::seconds>(finish - start).count();
			if (elapsed > 180) {
				run.elapsed_seconds = elapsed;
				break;
			}
		}
		
		std::cout << "\n";
		run.init.deleteEmptyRoutes();
		run.best.deleteEmptyRoutes();
		return run;
	}

	namespace details {

		Route PairInsertion(Request* r, Solution s, std::vector<EAV*> available_vehicles,Instance::Objective objective,bool includeCS) {
			std::random_device rd;
			int user = static_cast<int>(Instance::Objective::User);
			int owner = static_cast<int>(Instance::Objective::Owner);
			RandLib randlib(rd());
			Solution current_solution = s;
			Route empty_route;
			empty_route.batteryFeasible = false;
			empty_route.capacityFeasible = false;
			std::vector<Position> batset;
			std::vector<Position> capset;
			for (EAV* v : available_vehicles) {
				if (r->forbidden_vehicles[user].contains(v->id))
						continue;
				if (r->forbidden_vehicles[owner].contains(v->id))
						continue;
				bool batteryNotFound = true;
				size_t length = current_solution.routes[v].path.size();
				for (size_t i = 0; i < length-1; ++i)
				{
					if (inst.isForbiddenArc(current_solution.routes[v].path.at(i), r->origin)) continue;
					for (size_t j = i; j < length-1; ++j)
					{
						
						if (i == j) {
							if (inst.isForbiddenArc(r->destination, current_solution.routes[v].path.at(i + 1)))
								continue;
						}
						else {
							if (inst.isForbiddenArc(r->origin, current_solution.routes[v].path.at(i + 1))) break;
							if (inst.isForbiddenArc(r->destination, current_solution.routes[v].path.at(j + 1)) ||
								inst.isForbiddenArc(current_solution.routes[v].path.at(j),r->destination))
								continue;
							
						}
						double added_distance = current_solution.routes[v].getAddedDistance(r, i, j);
						if (current_solution.routes[v].batteryFeasibilityTest(r, i, j))
						{
							batset.emplace_back(v, i, j, nullptr, -1);
							batset.back().setAddedDistance(added_distance);
							batteryNotFound = false;
							if (current_solution.routes[v].isInsertionCapacityFeasible(r, i, j)) {
								capset.emplace_back(v, i, j, nullptr, -1);
								capset.back().setAddedDistance(added_distance);
							}
						}
					}
				}
				if (batteryNotFound && includeCS) {
					bool positionFound = false;
					std::vector<size_t> zero_load_positions;
					for (size_t i = current_solution.routes[v].path.size() - 2; i>0; i--)
					{

						if (current_solution.routes[v].loads.at(i) == 0) { 
							zero_load_positions.push_back(i);
						}
					}
					//Try inserting a charging station after zero-load nodes and then rescan the solution
					for (size_t pos : zero_load_positions) {
						CStation* min_s = current_solution.routes[v].findBestChargingStationAfter(pos);
						if (min_s != nullptr) {
							Node* cs_node = inst.nodes.at(min_s->id - 1);
							double stationAddedDistance = current_solution.routes[v].getAddedDistance(cs_node, pos);
							current_solution.routes[v].insertNode(cs_node, pos+1);
							current_solution.routes[v].updateMetrics();
							length = current_solution.routes[v].path.size();
							for (size_t i = 0; i < length-1; ++i)
							{
								if (inst.isForbiddenArc(current_solution.routes[v].path.at(i), r->origin)) continue;
								for (size_t j = i; j < length-1; ++j)
								{
									
									if (i == j) {
										if (inst.isForbiddenArc(r->destination, current_solution.routes[v].path.at(i + 1)))
											continue;
									}
									else {
										if (inst.isForbiddenArc(r->origin, current_solution.routes[v].path.at(i + 1))) break;
										if (inst.isForbiddenArc(r->destination, current_solution.routes[v].path.at(j + 1)) ||
											inst.isForbiddenArc(current_solution.routes[v].path.at(j), r->destination))
											continue;
									}
									double added_distance = current_solution.routes[v].getAddedDistance(r, i, j);
									if (current_solution.routes[v].batteryFeasibilityTest(r, i, j))
									{
										batset.emplace_back(v, i, j, min_s, pos);
										batset.back().setAddedDistance(stationAddedDistance + added_distance);
										if (current_solution.routes[v].isInsertionCapacityFeasible(r, i, j)) {
											capset.emplace_back(v, i, j, min_s, pos);
											capset.back().setAddedDistance(stationAddedDistance + added_distance);
											positionFound = true;
										}

									}
								}
							}
							current_solution.routes[v].removeNode(pos+1);
							current_solution.routes[v].updateMetrics();
							if (positionFound) break;

						}
					}
				}
			}
			if (batset.empty() || capset.empty()) { return empty_route; }
			else {
				Position min_pos;
				double min_cost=DBL_MAX;
				for (std::vector<Position>::iterator st = capset.begin(); st != capset.end(); ++st) {
					double cost = current_solution.getInsertionCost(r, *st,objective);
					if (cost < min_cost) {
						min_cost = cost;
						min_pos = *st;
					}
				}
				if (min_cost > 1) return empty_route;
				CStation* cs = min_pos.charging_station;
				if (cs != nullptr) current_solution.routes[min_pos.vehicle].insertNode(inst.nodes.at(cs->id - 1), min_pos.cs_pos + 1);
				current_solution.routes[min_pos.vehicle].insertRequest(r, min_pos.origin_pos + 1, min_pos.dest_pos + 1);
				current_solution.routes[min_pos.vehicle].updateMetrics();
				return current_solution.routes[min_pos.vehicle];
			}

		}

		double DistanceBetweenUsers(Request* a,Request* b) {
			return inst.getTravelTime(a->origin,b->origin) + inst.getTravelTime(a->destination,b->destination);
		}
		
		Solution Init1() {
			Solution solution;	
			std::random_device rd;
			RandLib randlib(rd());
			auto start = std::chrono::steady_clock::now();
			solution.AddDepots();
			std::vector<Request*> unassigned = inst.requests;
			std::shuffle(unassigned.begin(), unassigned.end(), rd);
			for (Request* request:unassigned) {
				Route inserted_route = PairInsertion(request, solution, inst.vehicles,Instance::Objective::NumberOfObjectives,true);
				if (inserted_route.isFeasible()) solution.addRoute(inserted_route);
				else solution.rejected.push_back(request); 
			}
			auto finish = std::chrono::steady_clock::now();
			double elapsed = std::chrono::duration_cast<std::chrono::seconds>(finish - start).count();
			std::cout << elapsed << std::endl;
			return solution;
		}

		Solution H1(int iter) {
            Solution solution;
			std::random_device rd;
			RandLib randlib(rd());
			solution.AddDepots();
			std::vector<Node*> requestNodes;
			requestNodes.insert(requestNodes.end(),inst.nodes.begin(), inst.nodes.begin() + 2 * inst.requests.size()-1);
			std::sort(requestNodes.begin(), requestNodes.end(), [](Node* n1, Node* n2) {return n1->latest < n2->latest; });
			for (size_t i = 0; i < requestNodes.size(); i++)
			{
				Node* node = requestNodes[i];
				EAV* v = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				int insertion_index = solution.routes[v].path.size() - 1;
				Route new_route = solution.routes[v];
				new_route.insertNode(node,insertion_index);
				solution.addRoute(new_route);
			
				if (node->isOrigin()) {
					Node* destination = inst.getRequest(node)->destination;
					insertion_index = randlib.randint(insertion_index + 1, solution.routes[v].path.size() - 1);
					requestNodes.erase(std::remove(requestNodes.begin(), requestNodes.end(), destination), requestNodes.end());
					Route route = solution.routes[v];
					route.insertNode(destination, insertion_index);
					route.updateMetrics();
					solution.addRoute(route);
				}
				else {
					Node* origin = inst.getRequest(node)->origin;
					insertion_index = randlib.randint(1,insertion_index);
					requestNodes.erase(std::remove(requestNodes.begin(), requestNodes.end(), origin), requestNodes.end());
					Route route = solution.routes[v];
					route.insertNode(origin, insertion_index);
					route.updateMetrics();
					solution.addRoute(route);
				}
			}
			solution.FixHardConstraints();
            return solution;
		}

		//Add battery feasibility constraint check needed when assigning the first m users.
		Solution Init2() {
			Solution solution;
			RandLib randlib(static_cast<unsigned int>(std::time(nullptr)));
			solution.AddDepots();

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
				Request* r = unassigned[i];
				int index_of_random_vehicle = randlib.randint(0, inst.vehicles.size() - 1);
				//Ensure that we have not inserted another request in the route, otherwise try again
				if (solution.routes[inst.vehicles.at(index_of_random_vehicle)].hasNoRequests()) {
					Route new_route = solution.routes[inst.vehicles.at(index_of_random_vehicle)];
					new_route.insertRequest(r, 1, 1);
					new_route.updateMetrics();
					if (new_route.isFeasible()) {
						solution.addRoute(new_route);
						unassigned.erase(unassigned.begin()+i);
						last_assigned_user[inst.vehicles.at(index_of_random_vehicle)] = r;
					}
					else i--;
				}
				else i--;
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
					Route new_route = PairInsertion(r, solution, { v },Instance::Objective::NumberOfObjectives,true);
					if (new_route.isFeasible()) {
						Solution new_solution = solution;
						new_solution.addRoute(new_route);
						if (new_solution.AchievementFunction(0.3) < solution.AchievementFunction(0.3)) {
							solution = new_solution;
							last_assigned_user[new_route.vehicle] = r;
							insertionFailed = false;
							break;
						}
					}
					
				}
				if (insertionFailed) {
					solution.rejected.push_back(r);
				}
				unassigned.erase(unassigned.begin());
			}
			return solution;
		}

		Solution Init4() {
			Solution solution;
			RandLib randlib(1);
			solution.AddDepots();
			std::vector<EAV*> eavs = inst.vehicles;

			//Sort the vehicles in ascending order of the shift duration
			std::sort(eavs.begin(), eavs.end(), [](EAV* v1, EAV* v2) {
				return v1->end_time - v1->start_time < v2->end_time - v2->start_time;
			});
			std::vector<Request*> unassigned = inst.requests;
			for (EAV* v : eavs) {
				std::shuffle(unassigned.begin(), unassigned.end(), randlib.Gen);
				for (size_t x = 0; x < unassigned.size(); x++) {
					Request* r = unassigned[x];
					std::vector<Position> feasible;
					for (size_t i = 0; i < solution.routes[v].path.size()-1; ++i)
					{
						if (inst.isForbiddenArc(solution.routes[v].path.at(i), r->origin)) continue;
						for (size_t j = i; j < solution.routes[v].path.size()-1; ++j)
						{
							if (i == j) {
								if (inst.isForbiddenArc(r->destination, solution.routes[v].path.at(i + 1)))
									continue;
							}
							else {
								if (inst.isForbiddenArc(r->origin, solution.routes[v].path.at(i + 1))) break;
								if (inst.isForbiddenArc(r->destination, solution.routes[v].path.at(j + 1)) ||
									inst.isForbiddenArc(solution.routes[v].path.at(j), r->destination))
									continue;

							}
							if (solution.routes[v].batteryFeasibilityTest(r, i, j) &&
								solution.routes[v].isInsertionCapacityFeasible(r, i, j))
								feasible.emplace_back(v, i, j, nullptr, -1);
						}
					}
					if (feasible.empty()) continue;
					else {
						Position min_pos = *feasible.begin();
						double min_cost = solution.getInsertionCost(r, min_pos);
						for (std::vector<Position>::iterator st = feasible.begin() + 1; st != feasible.end(); ++st) {
							double cost = solution.getInsertionCost(r, *st);
							if (cost < min_cost) {
								min_cost = cost;
								min_pos = *st;
							}
						}
						Route new_route = solution.routes[v];
						new_route.insertRequest(r, min_pos.origin_pos + 1, min_pos.dest_pos + 1);
						new_route.updateMetrics();
						solution.addRoute(new_route);
						unassigned.erase(unassigned.begin() + x);
					}
				}
			}
			while (!unassigned.empty()) {
				int random_request_index = randlib.randint(0, unassigned.size() - 1);
				Request* request = unassigned.at(random_request_index);
				Route inserted_route = PairInsertion(request, solution,inst.vehicles,Instance::Objective::NumberOfObjectives, true);
				if (inserted_route.isFeasible()) {
					Solution new_solution = solution;
					new_solution.addRoute(inserted_route);
					if (new_solution.AchievementFunction(0.3) < solution.AchievementFunction(0.3)) solution = new_solution;
					else solution.rejected.push_back(request);
				}
				else solution.rejected.push_back(request);
				unassigned.erase(unassigned.begin() + random_request_index);
			}
			return solution;
		}

		Solution moveStation(Solution s,NeighborChoice strategy)
		{
				Solution best = s;
				std::vector<EAV*> vehicles = inst.vehicles;
				std::vector<CStation*> charging_stations = inst.charging_stations;
				if (strategy == NeighborChoice::RANDOM) {
					std::shuffle(vehicles.begin(), vehicles.end(), std::random_device());
					std::shuffle(charging_stations.begin(), charging_stations.end(), std::random_device());
				}
				
				for (EAV* v : vehicles) {
					for (CStation* cs : charging_stations) {

						if (s.routes[v].assigned_cs[cs]) {
							Node* cs_node = inst.nodes.at(cs->id - 1);
							Route new_route = s.routes[v];
							new_route.removeNode(new_route.node_indices[cs_node]);
							new_route.updateMetrics();
							for (size_t i = 0; i < new_route.path.size() - 1; i++) {
								/* If the arc (sequence[i],sequence[i+1]) is a zero-load arc,
								we can insert a charging station in between its ends */
								if (new_route.loads[i] == 0) {
									new_route.insertNode(cs_node, i + 1);
									new_route.updateMetrics();
									if (new_route.isFeasible()) {
										Solution neighbor = s;
										neighbor.addRoute(new_route);
										if (strategy == NeighborChoice::RANDOM) return neighbor;
										if (neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
											if (strategy == NeighborChoice::FIRST) return neighbor;
											best = neighbor;
										}
									}
									new_route.removeNode(i + 1);
									new_route.updateMetrics();
								}
							}
						}
						s = best;
					}
				}
				return s;
		}

		Solution two_opt(Solution s,NeighborChoice strategy) {
			Solution best = s;
			std::random_device rd;
			RandLib randlib(rd());
			std::vector<EAV*> vehicles = inst.vehicles;
			if (strategy == NeighborChoice::RANDOM) std::shuffle(vehicles.begin(), vehicles.end(), std::random_device());
			for (size_t x = 0; x<vehicles.size()-1; x++) {
				EAV* v1 = vehicles[x];
				
				for (size_t y = x + 1; x < vehicles.size(); x++) {
					EAV* v2 = vehicles[y];
						for (size_t i = 0; i < s.routes[v1].path.size(); i++) {
							if (strategy == NeighborChoice::RANDOM) i = randlib.randint(0, s.routes[v1].path.size() - 1);
							if (s.routes[v1].loads.at(i) == 0 && i != 0 && i!=s.routes[v1].path.size()-1) {
								for (size_t j = 0; j < s.routes[v2].path.size(); j++) {
									if (strategy == NeighborChoice::RANDOM) j = randlib.randint(0, s.routes[v2].path.size() - 1);
									if (s.routes[v2].loads.at(j) == 0 && j != 0 && j != s.routes[v2].path.size() - 1) {
										Route new_r1(v1);
										Route new_r2(v2);

										//Reconnect the first part of the first route with the second part of the second one and vice versa.
										new_r1.path.insert(new_r1.path.end(),
											s.routes[v1].path.begin(),
											s.routes[v1].path.begin() + i+1
										);
										new_r1.path.insert(new_r1.path.end(),
											s.routes[v2].path.begin() + j + 1,
											s.routes[v2].path.end()
										);

										new_r2.path.insert(new_r2.path.end(),
											s.routes[v2].path.begin(),
											s.routes[v2].path.begin() + j+1
										);
										new_r2.path.insert(new_r2.path.end(),
											s.routes[v1].path.begin() + i + 1,
											s.routes[v1].path.end()
										);
										new_r1.path.back()=inst.getDepot(v1, "end");
										new_r2.path.front()=inst.getDepot(v2, "start");

										new_r1.updateMetrics();
										new_r2.updateMetrics();
										
									    Solution neighbor = s;
									    neighbor.addRoute(new_r1);
									    neighbor.addRoute(new_r2);
									    if (strategy == NeighborChoice::RANDOM) return neighbor;
									    if (new_r1.isFeasible() && new_r2.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
									    	if (strategy == NeighborChoice::FIRST) return neighbor;
									    	best = neighbor; 
									    }
										


									}
								}
							}
						}
				}
			}
			return best;
		}

		Solution cs_two_opt(Solution s) {
			Solution best = s;
			std::vector<Route> included_routes;

			for (CStation* cs : inst.charging_stations) {
				Node* cs_node = inst.nodes.at(cs->id - 1);
				//Find all routes that include the specific charging station.
				for (std::pair<EAV*, Route> pair : s.routes) {
					if (pair.second.node_indices.contains(cs_node)) {
						included_routes.push_back(pair.second);

					}
				}
				if (!included_routes.empty()) {
					//Take all included route pairs
					for (size_t i = 0; i < included_routes.size() - 1; i++)
					{
						for (size_t j = i + 1; j < included_routes.size(); j++)
						{
							//Find the charging station position in each route
							int first_route_index = included_routes[i].node_indices[cs_node];
							int second_route_index = included_routes[j].node_indices[cs_node];


							Route new_r1(included_routes[i].vehicle);
							Route new_r2(included_routes[j].vehicle);

							//Reconnect the first part of the first route with the second part of the second one and vice versa.
							new_r1.path.insert(new_r1.path.end(),
								included_routes.at(i).path.begin(),
								included_routes.at(i).path.begin() + first_route_index + 1
							);
							new_r1.path.insert(new_r1.path.end(),
								included_routes.at(j).path.begin() + second_route_index + 1,
								included_routes.at(j).path.end()
							);

							new_r2.path.insert(new_r2.path.end(),
								included_routes.at(j).path.begin(),
								included_routes.at(j).path.begin() + second_route_index + 1
							);
							new_r2.path.insert(new_r2.path.end(),
								included_routes.at(i).path.begin() + first_route_index + 1,
								included_routes.at(i).path.end()
							);
							new_r1.path.back() = inst.getDepot(new_r1.vehicle, "end");
							new_r2.path.front() = inst.getDepot(new_r2.vehicle, "start");

							new_r1.updateMetrics();
							new_r2.updateMetrics();
							if (new_r1.isFeasible() && new_r2.isFeasible()) {
								Solution neighbor = s;
								neighbor.addRoute(new_r1);
								neighbor.addRoute(new_r2);
								if (neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) best = neighbor;
							}

						}
					}
					included_routes.clear();
				}

			}
			return best;
		}

		Solution relocate(Solution s,NeighborChoice strategy) {
			std::random_device rd;
			RandLib randlib(rd());
			Solution best, current_solution;
			best = current_solution = s;

			for (EAV* v:inst.vehicles)
			{
				if (strategy == NeighborChoice::RANDOM) v = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (size_t i=1; i < s.routes[v].path.size()-1; i++)
				{
					if (strategy == NeighborChoice::RANDOM) i = randlib.randint(1, s.routes[v].path.size() - 2);
					if (s.routes[v].path[i]->isOrigin()) {
						Request* request = inst.getRequest(s.routes[v].path[i]);
						Route removal_route = s.routes[v];
						removal_route.removeRequest(request);
						removal_route.updateMetrics();
						std::vector<EAV*> vehicles = inst.vehicles;
						vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), v), vehicles.end());
						Route insertion_route = PairInsertion(request, s,vehicles,Instance::Objective::NumberOfObjectives,true);
						Solution neighbor = s;
						neighbor.addRoute(removal_route);
						neighbor.addRoute(insertion_route);
						if (strategy == NeighborChoice::RANDOM) return neighbor;
						if (insertion_route.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3))
						{
							if (strategy == NeighborChoice::FIRST) return neighbor;
							best = neighbor;
						}
						
						
					}
				}
			}
			return best;
		}

		Solution swapRequest(Solution s,NeighborChoice strategy) {
			Solution best = s;
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			for (EAV* v1: inst.vehicles){
				if (isRandom) v1 = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (EAV* v2 : inst.vehicles) {
					if (isRandom) v2 = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
					if (v1->id != v2->id) {
						for (Node* node1:s.routes[v1].path)
						{
							if (isRandom) node1 = s.routes[v1].path[randlib.randint(0, s.routes[v1].path.size() - 1)];
							if (node1->isOrigin()) {
								Request* r1 = inst.getRequest(node1);
								std::pair<size_t, size_t> r1_pos = s.routes[v1].getRequestPosition(r1);
								for (Node* node2 : s.routes[v2].path) {
									if (isRandom) node2 = s.routes[v2].path[randlib.randint(0, s.routes[v2].path.size() - 1)];
									if (node2->isOrigin()) {
										Request* r2 = inst.getRequest(node2);
										std::pair<size_t, size_t> r2_pos = s.routes[v2].getRequestPosition(r2);
										Route new_route_1 = s.routes[v1];
										Route new_route_2 = s.routes[v2];
										//Update metrics might be needed after the removals.
										new_route_1.removeRequest(r1);
										new_route_2.removeRequest(r2);
										new_route_1.insertRequest(r2, r1_pos.first, r1_pos.second-1);
										new_route_2.insertRequest(r1, r2_pos.first, r2_pos.second-1);
										new_route_1.updateMetrics();
										new_route_2.updateMetrics();
										
										Solution neighbor = s;
										neighbor.addRoute(new_route_1);
										neighbor.addRoute(new_route_2);
										if (isRandom) return neighbor;
										if (new_route_1.isFeasible() && new_route_2.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
											if (strategy == NeighborChoice::FIRST) return neighbor;
											best = neighbor;
										}
										
									}
								}
							}
						}
					}
				}
				
			}
			return best;
		}

		Solution swapRejected(Solution s,NeighborChoice strategy) {
			Solution best = s;
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			for (EAV* v:inst.vehicles) {
				if (isRandom) v = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (Node* node : s.routes[v].path) {
					if (isRandom) node = s.routes[v].path[randlib.randint(0, s.routes[v].path.size() - 1)];
					if (node->isOrigin()) {
						Request* assigned = inst.getRequest(node);
						std::pair<size_t, size_t> assigned_pos = s.routes[v].getRequestPosition(assigned);
						for (size_t i = 0; i<s.rejected.size();i++) {
							if (isRandom) i= randlib.randint(0, s.rejected.size() - 1);
							Request* r=s.rejected.at(i);
							Route new_route = s.routes[v];
							new_route.removeRequest(assigned);
							new_route.insertRequest(r, assigned_pos.first, assigned_pos.second-1);
							new_route.updateMetrics();
							
							Solution neighbor = s;
							neighbor.rejected.erase(std::remove(neighbor.rejected.begin(), neighbor.rejected.end(), r), neighbor.rejected.end());
							neighbor.rejected.push_back(assigned);
							neighbor.addRoute(new_route);
							if (isRandom) return neighbor;
							if (new_route.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
								if (strategy == NeighborChoice::FIRST) return neighbor;
								best = neighbor;
							}
						}
						
					}
				}
			}
			return best;
		}

		Solution swapRoute(Solution s,NeighborChoice strategy){
			Solution best = s;
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			for (EAV* v1:inst.vehicles) {
				if (isRandom) v1 = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (EAV* v2: inst.vehicles) {
					if (isRandom) v2 = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
					if (v1->id != v2->id) {
						Route r1 = s.routes[v1];
						Route r2 = s.routes[v2];
						r1.path[0] = inst.getDepot(v2, "start");
						r2.path[0] = inst.getDepot(v1, "start");
						r1.path.back() = inst.getDepot(v2, "end");
						r2.path.back() = inst.getDepot(v1, "end");
						r1.updateMetrics();
						r2.updateMetrics();
						Solution neighbor = s;
						neighbor.addRoute(r1);
						neighbor.addRoute(r2);
						if (isRandom) return neighbor;
						if (r1.isFeasible() && r2.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
							if (strategy == NeighborChoice::FIRST) return neighbor;
							best = neighbor;
						} 
					}
				}
			}
			return best;
		}

		Solution exchangeOrigin(Solution s, NeighborChoice strategy)
		{
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			Solution best = s;
			for (EAV* v:inst.vehicles)
			{
				if (isRandom) v = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (size_t i = 1; i<s.routes[v].path.size()-1;i++)
				{
					if (isRandom) i = randlib.randint(1, s.routes[v].path.size() - 2);
					
					if (s.routes[v].path[i]->isOrigin() && 
						!inst.isForbiddenArc(s.routes[v].path[i+1],s.routes[v].path[i])){
						Route route = s.routes[v];
						//Swap the two nodes
						Node* intermediate = route.path[i];
						route.path.at(i) = route.path.at(i + 1);
						route.path.at(i + 1) = intermediate;
						route.updateMetrics();
						Solution neighbor = s;
						neighbor.addRoute(route);
						if (isRandom) return neighbor;
						if (route.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
							if (strategy == NeighborChoice::FIRST) return neighbor;
							best = neighbor;
						}
					}
				}
				s=best;
			}
			return best;
		}

		Solution exchangeDestination(Solution s, NeighborChoice strategy) {
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			Solution best = s;
			for (EAV* v : inst.vehicles)
			{
				if (isRandom) v = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (size_t i = 1; i < s.routes[v].path.size() - 1; i++)
				{
					if (isRandom) i = randlib.randint(1, s.routes[v].path.size() - 2);

					if (s.routes[v].path[i]->isDestination() &&
						!inst.isForbiddenArc(s.routes[v].path[i], s.routes[v].path[i-1])) {
						Route route = s.routes[v];
						//Swap the two nodes
						Node* intermediate = route.path[i];
						route.path.at(i) = route.path.at(i-1);
						route.path.at(i-1) = intermediate;
						route.updateMetrics();
						if (route.capacityFeasible) {
							Solution neighbor = s;
							neighbor.addRoute(route);
							if (isRandom) return neighbor;
							if (route.batteryFeasible && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
								if (strategy == NeighborChoice::FIRST) return neighbor;
								best = neighbor;
							}
						}
					}
				}
				s = best;
			}
			return s;
		}

		Solution exchangeConsecutive(Solution s, NeighborChoice strategy) {
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			Solution best = s;
			for (EAV* v : inst.vehicles)
			{
				if (isRandom) v = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (size_t i = 1; i < s.routes[v].path.size() - 1; i++)
				{
					if (isRandom) i = randlib.randint(1, s.routes[v].path.size() - 2);

					if (s.routes[v].path[i]->isDestination() &&
						s.routes[v].path[i+1]->isOrigin()) {
						Route route = s.routes[v];
						//Swap the two nodes
						Node* intermediate = route.path[i];
						route.path.at(i) = route.path.at(i+1);
						route.path.at(i+1) = intermediate;
						route.updateMetrics();
						Solution neighbor = s;
						neighbor.addRoute(route);
						if (isRandom) return neighbor;
						if (route.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
							if (strategy == NeighborChoice::FIRST) return neighbor;
							best = neighbor;
						}
					}
				}
				s = best;
			}
			return s;
		}

		Solution swapNaturalSequence(Solution s,NeighborChoice strategy) {
			Solution best = s;
			bool isRandom = strategy == NeighborChoice::RANDOM;
			std::random_device rd;
			RandLib randlib(rd());
			for (EAV* v1 : inst.vehicles) {
				if (isRandom) v1 = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				for (EAV* v2 : inst.vehicles) {
					if (isRandom) v2 = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
					if (v1->id != v2->id)
					{
						for (std::pair<int,int> pair1 : s.routes[v1].natural_sequences) {
							if (isRandom) pair1 = s.routes[v1].natural_sequences[randlib.randint(0, s.routes[v1].natural_sequences.size() - 1)];
							for (std::pair<int,int> pair2 : s.routes[v2].natural_sequences) {
								if (isRandom) pair2 = s.routes[v2].natural_sequences[randlib.randint(0, s.routes[v2].natural_sequences.size() - 1)];
								Route r1(v1);
								Route r2(v2);
								r1.path.insert(r1.path.end(), s.routes[v1].path.begin(), s.routes[v1].path.begin() + pair1.first);
								r1.path.insert(r1.path.end(), s.routes[v2].path.begin()+pair2.first, s.routes[v2].path.begin() + pair2.second+1);
								r1.path.insert(r1.path.end(), s.routes[v1].path.begin()+pair1.second+1, s.routes[v1].path.end());

								r2.path.insert(r2.path.end(), s.routes[v2].path.begin(), s.routes[v2].path.begin() + pair2.first);
								r2.path.insert(r2.path.end(), s.routes[v1].path.begin() + pair1.first, s.routes[v1].path.begin() + pair1.second + 1);
								r2.path.insert(r2.path.end(), s.routes[v2].path.begin() + pair2.second + 1, s.routes[v2].path.end());

								
								r1.updateMetrics();
								r2.updateMetrics();
								Solution neighbor = s;
								neighbor.addRoute(r1);
								neighbor.addRoute(r2);
								if (isRandom) return neighbor;
								if (r1.isFeasible() && r2.isFeasible() && neighbor.AchievementFunction(0.3) < best.AchievementFunction(0.3)) {
									if (strategy == NeighborChoice::FIRST) return neighbor;
									best = neighbor;
								}
						    }
						}
					}
				}
			}

			return best;
		}

		Solution Destroy(Solution s, double removal_ratio, double randomness, Instance::Objective objective)
		{
			std::random_device rd;
			RandLib randlib(rd());
			Solution current_solution = s;
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * removal_ratio);

			std::vector<std::pair<Request*, std::pair<EAV*, double>>> requestPosition;
			for (EAV* v : inst.vehicles) {
				Route new_route;
				for (size_t i = 1; i < current_solution.routes[v].path.size() - 1; i++)
				{
					if (current_solution.routes[v].path[i]->isOrigin()) {
						Request* r = inst.getRequest(current_solution.routes[v].path[i]);
						double old_cost,new_cost,cost;
						switch (objective) {
						case Instance::Objective::System:
						{
							int j = current_solution.routes[v].node_indices[r->destination];
							double added_origin = inst.getTravelTime(current_solution.routes[v].path[i - 1], r->origin) +
								inst.getTravelTime(r->origin, current_solution.routes[v].path[i + 1]) -
								inst.getTravelTime(current_solution.routes[v].path[i - 1], current_solution.routes[v].path[i + 1]);
							double added_dest = inst.getTravelTime(current_solution.routes[v].path[j - 1], r->destination) +
								inst.getTravelTime(r->destination, current_solution.routes[v].path[j + 1]) -
								inst.getTravelTime(current_solution.routes[v].path[j - 1], current_solution.routes[v].path[j + 1]);
							cost = added_origin + added_dest;
							break;
						}
						case Instance::Objective::NumberOfObjectives:
						{
							old_cost = current_solution.AchievementFunction(0.3);
							new_route = current_solution.routes[v];
							new_route.removeRequest(r);
							new_route.updateMetrics();
							current_solution.addRoute(new_route);
							new_cost = current_solution.AchievementFunction(0.3);
							cost = old_cost - new_cost;
							break;
						}
						default:
						{
							old_cost = current_solution.objective_value[static_cast<int>(objective)];
							new_route = current_solution.routes[v];
							new_route.removeRequest(r);
							new_route.updateMetrics();
							current_solution.addRoute(new_route);
							new_cost = current_solution.objective_value[static_cast<int>(objective)];
							cost = old_cost - new_cost;
						}
						}
						std::pair<EAV*, double> vehicleAndCost = { v,cost};
						requestPosition.push_back ( {r,vehicleAndCost} );
						current_solution = s;
					}
				}
			}
			std::sort(requestPosition.begin(), requestPosition.end(),
				[](std::pair<Request*, std::pair<EAV*, double>> pair1, std::pair<Request*, std::pair<EAV*, double>> pair2)
				{ return pair1.second.second > pair2.second.second; }
			);
			for (size_t i = 0; i < removal_number; i++) {
				int index = floor(pow(randlib.unifrnd(0, 0.99), randomness) * requestPosition.size());
				EAV* vehicle = requestPosition[index].second.first;
				Request* req = requestPosition[index].first;
				Route route = current_solution.routes[vehicle];
				route.removeRequest(req);
				route.updateMetrics();
				current_solution.addRoute(route);
				current_solution.removed.push_back({req,vehicle});
				requestPosition.erase(requestPosition.begin() + index);
			}
			
			return current_solution;
		}

		Solution Repair(Solution s, Instance::Objective objective)
		{
			std::random_device rd;
			RandLib randlib(rd());
			for (Request* req : s.rejected) s.removed.push_back({ req,nullptr });
			s.rejected.clear();
			std::shuffle(s.removed.begin(), s.removed.end(), rd);
			for (size_t i = 0; i < s.removed.size(); i++) {
				bool notFound = false;
				auto pair = s.removed[i];
				std::vector<EAV*> vehicles = inst.vehicles;
				if (pair.second != nullptr) vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), pair.second), vehicles.end());
				Route new_route = PairInsertion(pair.first, s, vehicles,objective,true);
				if (new_route.isFeasible()) s.addRoute(new_route);
				else notFound = true;
				if (notFound && pair.second == nullptr) s.rejected.push_back(pair.first);
				else if (notFound && pair.second != nullptr) {
					new_route = PairInsertion(pair.first, s, { pair.second },objective,true);
					if (new_route.isFeasible()) s.addRoute(new_route);
					else s.rejected.push_back(pair.first);
				}
			}
			s.removed.clear();
			return s;
		}

		
	}
}


