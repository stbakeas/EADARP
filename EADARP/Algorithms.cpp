#include "Algorithms.h"
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

std::vector<std::vector<Node*>> PowerSet(const Route& route,const std::vector<size_t>& set, int n)
{
	bool* contain = new bool[n] {0};
	std::vector<std::vector<Node*>> power_set;
	
	for (int i = 0; i < n; i++)
	{
		std::vector<Node*> subset;
		contain[i] = 1;
		// All permutation
		do
		{
			for (int j = 0; j < n; j++) {
				if (contain[j]) {
					subset.push_back(route.path[set[j]]);
				}
			}
			
		} while (std::prev_permutation(contain, contain + n));
		if (subset.size()) power_set.push_back(subset);
	}
	return power_set;
}

namespace algorithms {

    #define getName(VariableName) # VariableName

	using namespace details;

	bool SimulatedAnnealingAcceptanceCriterion(const Solution& candidate,const Solution& incumbent, double current_temperature) {
		std::random_device rd;
		RandLib randlib(rd());
		double exponent = (incumbent.objectiveValue() - candidate.objectiveValue())/current_temperature;
		return randlib.rand() < std::exp(exponent);
	}

	int RouletteWheelSelection(const std::vector<Statistics>& stats, double weightSum) {
		std::random_device rd;
		RandLib randlib(rd());
		double randomNumber = randlib.rand();
		double offset(0.0);		
		for (int i = 0; i<stats.size(); i++) {
			offset+= stats[i].weight/weightSum;
			if (randomNumber < offset) return i;
		}
	}

	Run ALNS(const Solution& initial,unsigned int max_iterations,int max_seconds,double temperature_control, double removalRandomness, double removalPercentage,int segment_size,double reaction_factor)
	{
		double temperature, cooling_rate(1.0);
		std::array<double, 2> weightSum;
		std::array<std::vector<Statistics>,2> stats;

		stats[0].emplace_back(ZeroSplitRemoval, 0.25, 0, 0);
		stats[0].emplace_back(WorstRemoval, 0.25, 0, 0);
		stats[0].emplace_back(RandomRemoval, 0.25, 0, 0);
		stats[0].emplace_back(SimilarityRemoval, 0.25, 0, 0);
		weightSum[0] = 1.0;
		stats[1].emplace_back(RegretInsertion, 0.33, 0, 0);
		stats[1].emplace_back(GreedyInsertion, 0.34, 0, 0);
		stats[1].emplace_back(RandomInsertion, 0.33, 0, 0);
		weightSum[1] = 1.0;
			
		bool noFeasibleFound = true;
		double intra_percentage(0.05);
		std::random_device rd;
		RandLib randlib(rd());
     	std::array<double,2> removalArguments = { removalPercentage,removalRandomness };
		std::array<double,2> insertionArguments;
		Run run;
		auto start = std::chrono::steady_clock::now();
		Solution incumbent=run.best = run.init = initial;
		temperature = -temperature_control * run.best.objectiveValue() / log(0.5);
		cooling_rate = pow(0.01 / temperature, 1.0 / (max_iterations));

		unsigned int iter = 1, no_imp_iter=0;
		Solution s;
		while (iter <= max_iterations) {
			int removalIndex = RouletteWheelSelection(stats[0],weightSum[0]);
			int insertionIndex = RouletteWheelSelection(stats[1],weightSum[1]);

			s = stats[0][removalIndex].heuristic(incumbent,removalArguments);
			for (std::vector<EAV*>::iterator v = inst.vehicles.begin(); v != inst.vehicles.end(); ++v) {
				Route route = s.routes[*v];
				route.deleteRedundantChargingStations();
				if (route.batteryFeasible) s.addRoute(route);
			}
			s = stats[1][insertionIndex].heuristic(s, insertionArguments);		

			stats[0][removalIndex].attempts++;
			stats[1][insertionIndex].attempts++;

			if (dbl_round(s.objectiveValue(), 5) < dbl_round(incumbent.objectiveValue(), 5)) {
				incumbent = s;
				stats[0][removalIndex].score += 5;
				stats[1][insertionIndex].score += 5;
			}
			else if (SimulatedAnnealingAcceptanceCriterion(s, incumbent, temperature)) {
				incumbent = s;
				stats[0][removalIndex].score += 7;
				stats[1][insertionIndex].score += 7;
				
			}

			
			if (dbl_round(s.objectiveValue(), 5) < dbl_round(run.best.objectiveValue(), 5)) {
					run.best = incumbent = s;
					stats[0][removalIndex].score += 10;
					stats[1][insertionIndex].score += 10;
					run.best_iter = iter;
					printf("New best Found:%f, Rejected: %d\n",s.objectiveValue(),s.rejected.size());
					no_imp_iter = 0;
			}
			

			if (noFeasibleFound && run.best.rejected.empty()) {
				noFeasibleFound = false;
				printf("Feasible found! ");
				temperature = -temperature_control * run.best.objectiveValue() / log(0.5);
				//printf("Temperature: %f",temperature);
				cooling_rate = pow(0.01 / temperature, 1.0 / (max_iterations - iter));
				//printf(" Cooling Rate: %f\n",cooling_rate);
			}
			

			iter++;
			no_imp_iter++;
			temperature *= cooling_rate;

			if (iter % segment_size == 0) {
				
				for (int i = 0; i < 2; i++) {
					weightSum[i] = 0.0;
					for (auto it = stats[i].begin(); it != stats[i].end(); ++it) {
						if (it->attempts) {
							it->weight = (1.0 - reaction_factor)*it->weight + reaction_factor * it->score / it->attempts;
							it->attempts = 0;
						}
						weightSum[i] += it->weight;
						it->score = 0;
					}
				}
			}
			
			run.elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() / 1000.0;
			if (run.elapsed_seconds > max_seconds) break;
		}
		printf("Elapsed time: %f\n", run.elapsed_seconds);
		return run;
	}

	namespace details {

		std::vector<Position> InsertionNeighborhood(Request* r, Solution s, const std::vector<EAV*>& available_vehicles, bool includeCS)
		{
			std::random_device rd;
			RandLib randlib(rd());

			//Reserve size for feasible position vectors
			int vehicle_index = randlib.randint(0, available_vehicles.size() - 1);
			int example_route_size = s.routes[available_vehicles[vehicle_index]].path.size();
			int reserve_size = available_vehicles.size() * example_route_size * (example_route_size - 1) / 2;

			std::vector<std::pair<CStation*, Node*>> empty_station_vector;

			std::vector<Position> strong_batset;
			strong_batset.reserve(reserve_size);
			std::vector<Position> weak_batset;
			weak_batset.reserve(reserve_size);
			for (EAV* v : available_vehicles) {
				if (r->forbidden_vehicles.contains(v->id)) continue;
				size_t length = s.routes[v].path.size() - 1;
				bool batteryNotFound = true, timeAndCapacityFeasibleFound = false;
				
				for (size_t i = 0; i < length; ++i)
				{
				    
					if (inst.isForbiddenArc(s.routes[v].path[i], r->origin)) continue;
					
					for (size_t j = i; j < length; ++j)
					{
					    if (inst.isForbiddenArc(r->destination, s.routes[v].path[j + 1]))
					    	continue;
						
						if (i!=j) {
							if (inst.isForbiddenArc(r->origin, s.routes[v].path[i + 1])) break;
							if (inst.isForbiddenArc(s.routes[v].path[j], r->destination))
								continue;
						}
						
						if (s.routes[v].isInsertionCapacityFeasible(r, i, j) 
							&& s.routes[v].isInsertionTimeFeasible(r, i, j)) {
								timeAndCapacityFeasibleFound = true;
								if (s.routes[v].isInsertionBatteryFeasible(r, i, j, false)) {
									strong_batset.emplace_back(v, i, j,empty_station_vector);
									batteryNotFound = false;
								}
								if (s.routes[v].isInsertionBatteryFeasible(r, i, j, true)) {
									weak_batset.emplace_back(v, i, j,empty_station_vector);
									batteryNotFound = false;
								}
						}
						
					}
				}
				if (batteryNotFound && timeAndCapacityFeasibleFound  && includeCS) {

					bool positionFound = false;
					std::vector<size_t> zero_load_positions;
					for (int i = s.routes[v].path.size() - 2; i>=0; i--) if (s.routes[v].loads[i] == 0) zero_load_positions.push_back(i);
					std::vector<std::vector<Node*>> power_set = PowerSet(s.routes[v],zero_load_positions,std::min(zero_load_positions.size(),inst.charging_stations.size()));
					for (auto subset : power_set) {
						std::vector<std::pair<CStation*, Node*>> added_stations;
						for (Node* zero_load_node : subset) {
							int node_index = s.routes[v].node_indices[zero_load_node];
							CStation* min_s = 
								s.routes[v].findBestChargingStationAfter(node_index,s.stationVisits);
							if (min_s != nullptr && 
		                        s.routes[v].getAddedDistance(inst.nodes[min_s->id - 1], node_index) <= s.routes[v].FTS[node_index + 1]) {
								Node* cs_node = inst.nodes[min_s->id - 1];
								s.routes[v].insertNode(cs_node, node_index + 1);
								s.routes[v].updateMetrics();
								added_stations.emplace_back(min_s, zero_load_node);
							}
							
						}
						if (s.routes[v].isFeasible() && !added_stations.empty()) {
							length = s.routes[v].path.size() - 1;
							for (size_t i = 0; i < length; ++i)
							{
								
								if (inst.isForbiddenArc(s.routes[v].path[i], r->origin)) continue;
								
								for (size_t j = i; j <length; ++j)
								{
									if (inst.isForbiddenArc(r->destination, s.routes[v].path[j + 1]))
											continue;
									
									if(i!=j) {
										if (inst.isForbiddenArc(r->origin, s.routes[v].path[i + 1])) break;
										if  (inst.isForbiddenArc(s.routes[v].path[j], r->destination))
											continue;
									}
									if (s.routes[v].isInsertionTimeFeasible(r, i, j) && 
										s.routes[v].isInsertionCapacityFeasible(r, i, j) && 
										s.routes[v].isInsertionBatteryFeasible(r, i, j, true))
									{	
										weak_batset.emplace_back(v, i, j,added_stations);
										positionFound = true;
									}
								}
							}
						}
						for (const auto& [station,node]:added_stations) 
							s.routes[v].removeNode(s.routes[v].node_indices[inst.nodes[station->id-1]]);
						if (!added_stations.empty()) s.routes[v].updateMetrics();
						if (positionFound) break;

					}
				}
			}
			return strong_batset.empty()?weak_batset:strong_batset;
		}

		Solution Init1() {
			Solution solution;	
			std::random_device rd;
			RandLib randlib(rd());
			solution.AddDepots();
			std::vector<Request*> unassigned = inst.requests;
			std::sort(unassigned.begin(), unassigned.end(), [](Request* r1, Request* r2) {return r1->origin->earliest < r2->origin->earliest; });
			for (Request* request:unassigned) {
				std::vector<Position> possibleMoves = InsertionNeighborhood(request, solution, inst.vehicles,true);
				Position min_pos;
				double min_cost(DBL_MAX);

				//Greedy Insertion
				for (std::vector<Position>::iterator st = possibleMoves.begin(); st != possibleMoves.end(); ++st) {
					double cost = solution.getInsertionCost(request, *st);
					if (cost < min_cost) {
						min_cost = cost;
						min_pos = *st;
					}
				}
				if (min_cost != DBL_MAX) {
					Route inserted_route = solution.routes[min_pos.vehicle];
					for (const auto& [station, node] : min_pos.cs_pos) 
						inserted_route.insertNode(inst.nodes[station->id - 1], inserted_route.node_indices[node] + 1);
					inserted_route.insertRequest(request, min_pos.origin_pos + 1, min_pos.dest_pos + 1);
					inserted_route.updateMetrics(true);
					solution.addRoute(inserted_route);
				}
				else solution.rejected.push_back(request); 
			}
			return solution;
		}

		void moveStation(Solution& s,NeighborChoice strategy)
		{
				Solution best = s;
				std::vector<EAV*> vehicles = inst.vehicles;
				std::vector<CStation*> charging_stations = inst.charging_stations;
				if (strategy == NeighborChoice::RANDOM) {
					std::shuffle(vehicles.begin(), vehicles.end(), std::random_device());
					std::shuffle(charging_stations.begin(), charging_stations.end(), std::random_device());
				}
				
				for (EAV* v : vehicles) {
					for (const auto& [cs,amount] : s.routes[v].desiredAmount) {
							Node* cs_node = inst.nodes[cs->id - 1];
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
										if (strategy == NeighborChoice::RANDOM) {
											s = neighbor;
											return;
										}
										if (neighbor.objectiveValue() < best.objectiveValue()) {
											if (strategy == NeighborChoice::FIRST) {
												s = neighbor;
												return;
											}
											best = neighbor;
										}
									}
									new_route.removeNode(i + 1);
									new_route.updateMetrics();
								}
							}
						s = best;
					}
				}
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
										new_r2.path[0]=inst.getDepot(v2, "start");

										new_r1.updateMetrics();
										new_r2.updateMetrics();
										
									    Solution neighbor = s;
									    neighbor.addRoute(new_r1);
									    neighbor.addRoute(new_r2);
									    
									    if (new_r1.isFeasible() && new_r2.isFeasible()) {
											if (strategy == NeighborChoice::RANDOM) return neighbor;
											if (neighbor.objectiveValue() < best.objectiveValue()) {
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

		Solution cs_two_opt(Solution s) {
			Solution best = s;
			std::vector<Route> included_routes;
			included_routes.reserve(inst.vehicles.size());

			for (CStation* cs : inst.charging_stations) {
				Node* cs_node = inst.nodes.at(cs->id - 1);
				//Find all routes that include the specific charging station.
				for (std::pair<EAV*, Route> pair : s.routes) {
					if (pair.second.node_indices.contains(cs_node)) included_routes.push_back(pair.second);
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
							new_r2.path[0] = inst.getDepot(new_r2.vehicle, "start");

							new_r1.updateMetrics();
							new_r2.updateMetrics();
							if (new_r1.isFeasible() && new_r2.isFeasible()) {
								Solution neighbor = s;
								neighbor.addRoute(new_r1);
								neighbor.addRoute(new_r2);
								if (neighbor.objectiveValue() < best.objectiveValue()) best = neighbor;
							}

						}
					}
					included_routes.clear();
					included_routes.reserve(inst.vehicles.size());
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
										if (new_route_1.isFeasible() && new_route_2.isFeasible()){
											if (isRandom) return neighbor;
											if (neighbor.objectiveValue() < best.objectiveValue()) {
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
							
							if (new_route.isFeasible()){
								if (isRandom) return neighbor;
								if (neighbor.objectiveValue() < best.objectiveValue())
								{
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
						if (r1.isFeasible() && r2.isFeasible() && neighbor.objectiveValue() < best.objectiveValue()) {
							if (strategy == NeighborChoice::FIRST) return neighbor;
							best = neighbor;
						} 
					}
				}
			}
			return best;
		}

		void exchangeOrigin(Solution& s, NeighborChoice strategy)
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
						!inst.isForbiddenArc(s.routes[v].path[i+1],s.routes[v].path[i]) && 
						!inst.isForbiddenArc(s.routes[v].path[i-1], s.routes[v].path[i+1]) &&
						!inst.isForbiddenArc(s.routes[v].path[i], s.routes[v].path[i+2])){
						Route route = s.routes[v];
						//Swap the two nodes
						Node* intermediate = route.path[i];
						route.path[i] = route.path[i + 1];
						route.path[i + 1] = intermediate;
						route.updateMetrics();
						
						if (route.isFeasible()) {
							Solution neighbor = s;
							neighbor.addRoute(route);
							if (isRandom) {
								s = neighbor;
								return;
							}
							if (neighbor.objectiveValue() < best.objectiveValue()){
								if (strategy == NeighborChoice::FIRST) {
									s = neighbor;
									return;
								}
								best = neighbor;
							}
							
						}
					}
				}
				s=best;
			}
		}

		void exchangeDestination(Solution& s, NeighborChoice strategy) {
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
						!inst.isForbiddenArc(s.routes[v].path[i], s.routes[v].path[i-1]) &&
						!inst.isForbiddenArc(s.routes[v].path[i-2], s.routes[v].path[i]) &&
						!inst.isForbiddenArc(s.routes[v].path[i-1], s.routes[v].path[i+1])) {
						Route route = s.routes[v];
						//Swap the two nodes
						Node* intermediate = route.path[i];
						route.path[i] = route.path[i-1];
						route.path[i-1] = intermediate;
						route.updateMetrics();
						if (route.isFeasible()) {
							Solution neighbor = s;
							neighbor.addRoute(route);
							if (isRandom) {
								s = neighbor;
								return;
							}
							if (neighbor.objectiveValue() < best.objectiveValue()) {
								if (strategy == NeighborChoice::FIRST) {
									s = neighbor;
									return;
								}
								best = neighbor;
							}

						}
					}
				}
				s = best;
			}
		}

		void exchangeConsecutive(Solution& s, NeighborChoice strategy) {
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
						s.routes[v].path[i+1]->isOrigin() &&
						!inst.isForbiddenArc(s.routes[v].path[i + 1], s.routes[v].path[i]) &&
						!inst.isForbiddenArc(s.routes[v].path[i - 1], s.routes[v].path[i + 1]) &&
						!inst.isForbiddenArc(s.routes[v].path[i], s.routes[v].path[i + 2])) {
						Route route = s.routes[v];
						//Swap the two nodes
						Node* intermediate = route.path[i];
						route.path[i] = route.path[i+1];
						route.path[i+1] = intermediate;
						route.updateMetrics();
						if (route.isFeasible()) {
							Solution neighbor = s;
							neighbor.addRoute(route);
							if (isRandom) {
								s = neighbor;
								return;
							}
							if (neighbor.objectiveValue() < best.objectiveValue()) {
								if (strategy == NeighborChoice::FIRST) {
									s = neighbor;
									return;
								}
								best = neighbor;
							}

						}
					}
				}
				s = best;
			}
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
								
								if (r1.isFeasible() && r2.isFeasible()) {
									if (isRandom) return neighbor;
									if (neighbor.objectiveValue() < best.objectiveValue()) {
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

#pragma region Destroy Operators

		Solution WorstRemoval(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);

			std::vector<std::pair<Request*, std::pair<EAV*, double>>> requestPosition;
			requestPosition.reserve(inst.requests.size());
			for (EAV* v : inst.vehicles) {
				
				for (Request* r: s.routes[v].requests)
				{
					Route new_route = s.routes[v];
					new_route.removeRequest(r);
					new_route.updateMetrics();
					requestPosition.emplace_back(r,std::make_pair(v, 0.75 * (s.routes[v].travel_distance - new_route.travel_distance) +
						0.25 * (s.routes[v].excess_ride_time - new_route.excess_ride_time)));
				}
			}
			std::sort(requestPosition.begin(), requestPosition.end(),
				[](std::pair<Request*, std::pair<EAV*, double>> pair1, std::pair<Request*, std::pair<EAV*, double>> pair2)
				{ return pair1.second.second > pair2.second.second; }
			);
			for (size_t i = 0; i < removal_number; i++) {
				int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * requestPosition.size());
				Route route = s.routes[requestPosition[index].second.first];
				route.removeRequest(requestPosition[index].first);
				route.updateMetrics(true);
				if (!route.isFeasible()) printf("Infeasible Removal? \n");
				s.removed.push_back(requestPosition[index].first);
				s.addRoute(route);
				requestPosition.erase(requestPosition.begin() + index);
			}

			return s;
		}

		Solution RandomRemoval(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);

			int i = 0;
			while(i<removal_number) {
				EAV* random_vehicle = inst.vehicles[randlib.randint(0,inst.vehicles.size()-1)];
				if (!s.routes[random_vehicle].hasNoRequests()) {
					Route route = s.routes[random_vehicle];
					Request* req = route.requests[randlib.randint(0, route.requests.size() - 1)];;
					route.removeRequest(req);
					route.updateMetrics(true);
					if (route.isFeasible()){}
					else printf("Infeasible Removal? \n");
					s.removed.push_back(req);
					s.addRoute(route);
					i++;
				}
			}

			return s;
		}

		Solution ZeroSplitRemoval(Solution s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (!removal_number) return s;
			std::vector<std::pair<std::vector<Request*>, std::pair<EAV*, double>>> zssPosition;
			zssPosition.reserve(inst.requests.size());
			for (EAV* v : inst.vehicles) {
				for (std::pair<int,int> pair : s.routes[v].natural_sequences) {
					std::vector<Request*> removedRequests;
					double cost(0.0);
					for (int i = pair.first; i < pair.second; i++) {
						if (s.routes[v].path[i]->isOrigin()) removedRequests.push_back(inst.getRequest(s.routes[v].path[i]));
						cost+=inst.getTravelTime(s.routes[v].path[i], s.routes[v].path[i + 1])+s.routes[v].waiting_times[i];
					}
					zssPosition.emplace_back(removedRequests,std::make_pair(v,cost));
				}
			}

			std::sort(zssPosition.begin(), zssPosition.end(),
				[](std::pair<std::vector<Request*>, std::pair<EAV*, double>> pair1, std::pair<std::vector<Request*>, std::pair<EAV*, double>> pair2)
				{ return pair1.second.second > pair2.second.second; }
			);


			int i = 0;
			while (i < removal_number) {
				int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * zssPosition.size());
				EAV* vehicle = zssPosition[index].second.first;
				
				Route route = s.routes[vehicle];
				for (Request* req : zssPosition[index].first) {
					route.removeRequest(req);
					s.removed.push_back(req);
					i++;
				}

				route.updateMetrics(true);
				s.addRoute(route);
				zssPosition.erase(zssPosition.begin() + index);
			}
			return s;
		}

		Solution EntireRouteRemoval(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (!removal_number) return s;
			int i = 0;
			while (i < removal_number) {
				EAV* vehicle = inst.vehicles[randlib.randint(0, inst.vehicles.size()-1)];
				if (!s.routes[vehicle].hasNoRequests()) {
					for (Request* req : s.routes[vehicle].requests) s.removed.push_back(req);
					i += s.routes[vehicle].requests.size();

					Route route(vehicle);
					route.insertNode(inst.getDepot(vehicle, "start"), 0);
					route.insertNode(inst.getDepot(vehicle, "end"), 1);
					route.updateMetrics(true);
					s.addRoute(route);
				}
			}
			return s;

		}

		Solution SimilarityRemoval(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (removal_number)
			{
				std::vector<Assignment> assignments;
				assignments.reserve(inst.requests.size());
				for (EAV* v : inst.vehicles) {
					for (Request* r : s.routes[v].requests) assignments.emplace_back(r, v);
				}

				Request* removed_req;
				if (s.rejected.empty()) {
					int randomIndex = randlib.randint(0, assignments.size() - 1);
					s.removed.push_back(assignments[randomIndex].first);
					Route random_route = s.routes[assignments[randomIndex].second];
					removed_req = assignments[randomIndex].first;
					assignments.erase(assignments.begin() + randomIndex);
					random_route.removeRequest(removed_req);
					random_route.updateMetrics(true);
					if (!random_route.isFeasible()) { 
						std::cout << "Infeasible Similarity Removal?\n"; 
					}
					s.addRoute(random_route);
					removal_number--;
				}
				else removed_req = s.rejected[randlib.randint(0, s.rejected.size() - 1)];
				

				std::sort(assignments.begin(), assignments.end(), [removed_req](std::pair<Request*,EAV*> pair1,std::pair<Request*,EAV*> pair2) {
					return inst.similarity[removed_req->origin->id - 1][pair1.first->origin->id - 1] <
						inst.similarity[removed_req->origin->id - 1][pair2.first->origin->id - 1];
				});
				size_t i = 0;
				while (i < removal_number) {
					int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * assignments.size());
					Route route = s.routes[assignments[index].second];
					route.removeRequest(assignments[index].first);
					route.updateMetrics(true);
					if (!route.isFeasible()) printf("Infeasible Removal? \n");
					s.addRoute(route);
					s.removed.push_back(assignments[index].first);
					assignments.erase(assignments.begin() + index);
					i++;
				}
			}
			return s;
			
		}

#pragma endregion 

#pragma region Repair Operators

		Solution GreedyInsertion(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			std::copy(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.clear();
			while (!s.removed.empty()) {
				Position bestMove;
				double bestCost(DBL_MAX) ;
				Request* minUser=nullptr;
				for (Request* request : s.removed) {
					std::vector<Position> possibleMoves = InsertionNeighborhood(request, s, inst.vehicles,true);
					Position min_pos;
					double min_cost(DBL_MAX);
					for (Position move : possibleMoves) {
						double cost = s.getInsertionCost(request, move);
						if (cost < min_cost) {
							min_cost = cost;
							min_pos = move;
						}
					}
					if (min_cost != DBL_MAX && min_cost<bestCost){
						bestMove = min_pos;
						bestCost = min_cost;
						minUser = request;
					}
				}
				if (bestCost == DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.clear();
				}
				else {
					Route new_route = s.routes[bestMove.vehicle];
					for (const auto& [station, node] : bestMove.cs_pos)
						new_route.insertNode(inst.nodes[station->id - 1], new_route.node_indices[node] + 1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					if (!new_route.isFeasible()) std::cout << "Infeasible Insertion?\n";
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.addRoute(new_route);
				}
			}
			return s;
		}

		Solution LeastOptionsInsertion(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			while (!s.rejected.empty()) {
				std::vector<Position> leastMoves;
				size_t min_size = SIZE_MAX;
				Request* minUser=nullptr;
				for (Request* request : s.rejected) {
					std::vector<Position> possibleMoves = InsertionNeighborhood(request, s, inst.vehicles,true);
					std::vector<Position> feasibleMoves;
					feasibleMoves.reserve(possibleMoves.size());
					for (size_t i = 0; i < possibleMoves.size(); i++) {
						possibleMoves[i].cost = s.getInsertionCost(request, possibleMoves[i]);
						if (possibleMoves[i].cost != DBL_MAX) feasibleMoves.push_back(possibleMoves[i]);
					}
					if (feasibleMoves.empty()) continue;
					else {
						std::sort(feasibleMoves.begin(), feasibleMoves.end(), [](Position p1,Position p2) {
							return p1.cost < p2.cost;
						});
						if (feasibleMoves.size() < min_size) {
							leastMoves = feasibleMoves; 
							min_size = feasibleMoves.size();
							minUser = request;
						}
						else if (feasibleMoves.size()==min_size) {
							if (feasibleMoves[0].cost < leastMoves[0].cost) {
								leastMoves = feasibleMoves;
								minUser = request;
							}
							
						}
					}
				}
				if (min_size==SIZE_MAX) {
					rejectedReinsertion(s);
					return s;
				}
				else {
					Position bestMove = leastMoves[floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * leastMoves.size())];
					Route new_route = s.routes[bestMove.vehicle];
					for (const auto& [station, node] : bestMove.cs_pos) 
						new_route.insertNode(inst.nodes[station->id - 1], new_route.node_indices[node] + 1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					if (!new_route.isFeasible()) std::cout << "Infeasible Insertion?\n";
					s.addRoute(new_route);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
				}

			}
			return s;
		}

		Solution SortedRandomizedInsertion(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			std::sort(s.removed.begin(), s.removed.end(), [](Request* r1, Request* r2) {
				return r1->origin->earliest < r2->origin->earliest;
			});
			for (int i = 0; i < s.rejected.size(); i++) {
				Request* request = s.rejected[i];
				std::vector<Position> possibleMoves = InsertionNeighborhood(request, s, inst.vehicles, true);
				std::vector<Position> feasibleMoves;
				for (Position move : possibleMoves) {
					double cost = s.getInsertionCost(request, move);
					if (cost != DBL_MAX) feasibleMoves.push_back(move);
				}
				if (feasibleMoves.size()) {
					Position bestMove = feasibleMoves[floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * feasibleMoves.size())];
					Route new_route = s.routes[bestMove.vehicle];
					for (const auto& [station, node] : bestMove.cs_pos)
						new_route.insertNode(inst.nodes[station->id - 1], new_route.node_indices[node] + 1);
					new_route.insertRequest(request, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					if (!new_route.isFeasible()) std::cout << "Infeasible Insertion?\n";
					s.addRoute(new_route);
					s.rejected.erase(s.rejected.begin() + i);
				}
			}
			rejectedReinsertion(s);
			return s;
		}
			
		Solution RandomInsertion(Solution s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			std::copy(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.clear();
			std::shuffle(s.removed.begin(), s.removed.end(), rd);
			for (int i = 0; i < s.removed.size(); i++) {
				Request* request = s.removed[i];
				std::vector<Position> possibleMoves = InsertionNeighborhood(request, s, inst.vehicles, true);
				std::vector<Position> feasibleMoves;
				std::copy_if(possibleMoves.begin(), possibleMoves.end(), std::back_inserter(feasibleMoves),
					[&s, request](Position move) {return s.getInsertionCost(request, move) != DBL_MAX; }
				);
				if (feasibleMoves.size()) {
					Position bestMove = feasibleMoves[randlib.randint(0,feasibleMoves.size()-1)];
					Route new_route = s.routes[bestMove.vehicle];
					for (const auto& [station, node] : bestMove.cs_pos)
						new_route.insertNode(inst.nodes[station->id - 1], new_route.node_indices[node] + 1);
					new_route.insertRequest(request, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					if (!new_route.isFeasible()) std::cout << "Infeasible Insertion?\n";
					s.removed.erase(s.removed.begin() + i);
					s.addRoute(new_route);
				}
			}
			rejectedReinsertion(s);
			s.removed.clear();
			return s;
		}

		Solution RegretInsertion(Solution s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			int kappa = randlib.randint(2,inst.vehicles.size());
			std::copy(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.clear();
			while (!s.removed.empty()) {
				Position bestMove;
				bestMove.cost = DBL_MAX;
				double max_regret = -DBL_MAX;
				Request* minUser=nullptr;
				for (Request* request : s.removed) {
					std::vector<Position> possibleMoves = InsertionNeighborhood(request, s, inst.vehicles,true);
					std::vector<Position> feasibleMoves;
					feasibleMoves.reserve(possibleMoves.size());
					std::copy_if(possibleMoves.begin(), possibleMoves.end(), std::back_inserter(feasibleMoves),
						[&s, request](Position move) {return s.getInsertionCost(request, move) != DBL_MAX; }
					);
					if (feasibleMoves.empty()) continue;
					else {
						std::vector<std::pair<int, Position>> RouteBestMove;
						for (EAV* v : inst.vehicles) {
							Position routeBest;
							routeBest.cost = DBL_MAX;
							for (Position move : feasibleMoves) {
								if (v->id == move.vehicle->id) {
									if (move.cost < routeBest.cost) {
										routeBest = move;
									}
								}
							}
							RouteBestMove.emplace_back(v->id, routeBest);
						}

						std::sort(RouteBestMove.begin(), RouteBestMove.end(), [](std::pair<int, Position> p1, std::pair<int, Position> p2) 
						{return p1.second.cost < p2.second.cost;});


						double regret(0.0);
						int counter = 1;
						for (auto vehicleMovePair : RouteBestMove) {
							if (counter > kappa) break;
							regret+=vehicleMovePair.second.cost - RouteBestMove[0].second.cost;
							counter++;
						}
						if (max_regret < regret) {
							bestMove = RouteBestMove[0].second;
							max_regret = regret;
							minUser = request;
						}
						else if (regret == max_regret) {
							if (RouteBestMove[0].second.cost < bestMove.cost) {
								bestMove = RouteBestMove[0].second;
								minUser = request;
							}
						}
					}
				}
				if (bestMove.cost==DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.clear();
				}
				else {
					Route new_route = s.routes[bestMove.vehicle];
					for (const auto& [station, node] : bestMove.cs_pos)
						new_route.insertNode(inst.nodes[station->id - 1], new_route.node_indices[node] + 1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					if (!new_route.isFeasible()) std::cout << "Infeasible Insertion?\n";
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.addRoute(new_route);
				}

			}
			return s;
		}

#pragma endregion

		void rejectedReinsertion(Solution& s)
		{
			// Request ID, Vehicle It Is Assigned To
			std::unordered_map<int, EAV*> assignment;
			for (const auto& [vehicle, route] : s.routes) {
				for (Request* req : route.requests) {
					assignment.emplace(req->origin->id,vehicle);
				}
			}

			std::vector<Request*> assigned;
			assigned.reserve(inst.requests.size());
			for (const auto& [reqId, vehicle] : assignment)
				assigned.push_back(inst.getRequest(inst.nodes[reqId - 1]));

			for (Request* removedRequest:s.removed) {
				bool insertionFailed = true;

				std::sort(assigned.begin(), assigned.end(), [removedRequest](Request* a, Request* b) {
					return inst.similarity[removedRequest->origin->id - 1][a->origin->id - 1] <
						inst.similarity[removedRequest->origin->id - 1][b->origin->id - 1];
					});

				for (Request* req : assigned) {
					Route route = s.routes[assignment[req->origin->id]];
					std::pair<size_t, size_t> position = route.getRequestPosition(req);
					route.removeRequest(req);
					route.insertRequest(removedRequest, position.first, position.second - 1);
					
					route.updateMetrics();
					if (route.isFeasible()) {
						Solution intermediate = s;
						intermediate.addRoute(route);
						std::vector<Position> possibleMoves = InsertionNeighborhood(req, intermediate, inst.vehicles, true);
						Position bestMove;
						double min_cost = DBL_MAX;
						for (Position move : possibleMoves) {
							double cost = intermediate.getInsertionCost(req, move);
							if (cost < min_cost) {
								min_cost = cost;
								bestMove = move;
							}
						}
						if (min_cost != DBL_MAX) {
							Route new_route = intermediate.routes[bestMove.vehicle];
							for (const auto& [station, node] : bestMove.cs_pos) 
								new_route.insertNode(inst.nodes[station->id - 1], new_route.node_indices[node] + 1);
							new_route.insertRequest(req, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
							new_route.updateMetrics(true);
							assignment[req->origin->id] = new_route.vehicle;
							assignment[removedRequest->origin->id] = route.vehicle;
							if (!new_route.isFeasible()) std::cout << "Infeasible Insertion?\n";
							intermediate.addRoute(new_route);
							s = intermediate;
							insertionFailed = false;
							break;
						}
					}
				}
				if (insertionFailed) s.rejected.push_back(removedRequest);
			}
		}

	}
}



