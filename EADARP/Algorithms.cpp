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

namespace algorithms {

    #define getName(VariableName) # VariableName

	using namespace details;

	bool SimulatedAnnealingAcceptanceCriterion(const Solution& candidate,const Solution& incumbent, double current_temperature,RandLib& randlib) {
		return randlib.rand() < std::exp((incumbent.objectiveValue() - candidate.objectiveValue()) / current_temperature);
	}

	int RouletteWheelSelection(const std::vector<Statistics>& stats, double weightSum, RandLib& randlib) {
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

		unsigned int no_imp_iter=0;
		Solution s;
		for (unsigned int iter = 1; iter <= max_iterations; ++iter) {
			int removalIndex = RouletteWheelSelection(stats[0],weightSum[0],randlib);
			int insertionIndex = RouletteWheelSelection(stats[1],weightSum[1],randlib);
			s = incumbent;
			stats[0][removalIndex].heuristic(s,removalArguments);
			for (std::vector<EAV*>::iterator v = inst.vehicles.begin(); v != inst.vehicles.end(); ++v) {
				Route route = s.routes[*v];
				if (route.deleteRedundantChargingStations()) {
					route.updateMetrics();
					if (route.batteryFeasible) s.addRoute(route);
				}
			}
			stats[1][insertionIndex].heuristic(s, insertionArguments);		

			stats[0][removalIndex].attempts++;
			stats[1][insertionIndex].attempts++;

			if (dbl_round(s.objectiveValue(), 2) < dbl_round(incumbent.objectiveValue(), 2)) {
				incumbent = s;
				stats[0][removalIndex].score += 5;
				stats[1][insertionIndex].score += 5;
			}
			else if (SimulatedAnnealingAcceptanceCriterion(s, incumbent, temperature,randlib)) {
				incumbent = s;
				stats[0][removalIndex].score += 7;
				stats[1][insertionIndex].score += 7;
				
			}

			
			if (dbl_round(s.objectiveValue(), 2) < dbl_round(run.best.objectiveValue(), 2)) {
					run.best = incumbent = std::move(s);
					stats[0][removalIndex].score += 10;
					stats[1][insertionIndex].score += 10;
					run.best_iter = iter;
					printf("New best Found:%f, Rejected: %d\n",s.objectiveValue(),s.rejected.size());
					no_imp_iter = 0;
			}
			

			if (noFeasibleFound && run.best.rejected.empty()) {
				noFeasibleFound = false;
				printf("Feasible found!\n");
			}
			
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

		std::vector<Position> InsertionNeighborhood(Request* r, Solution& s, const std::vector<EAV*>& available_vehicles, bool includeCS)
		{
			std::random_device rd;
			RandLib randlib(rd());

			//Reserve size for feasible position vectors
			int vehicle_index = randlib.randint(0, available_vehicles.size() - 1);
			int example_route_size = s.routes[available_vehicles[vehicle_index]].path.size();
			int reserve_size = available_vehicles.size() * example_route_size * (example_route_size - 1) / 2;

			std::vector<Position> batset;
			batset.reserve(reserve_size);
			for (EAV* v : available_vehicles) {
				Route route = s.routes[v];
				route.BasicScheduling();
				if (r->forbidden_vehicles.contains(v->id)) continue;
				size_t length = route.path.size() - 1;
				bool batteryNotFound = true, timeAndCapacityFeasibleFound = false;
				
				for (size_t i = 0; i < length; i++)
				{
				    
					if (inst.isForbiddenArc(route.path[i], r->origin)) continue;
					
					for (size_t j = i; j < length; j++)
					{
					    if (inst.isForbiddenArc(r->destination, route.path[j + 1]))
					    	continue;
						
						if (i!=j) {
							if (inst.isForbiddenArc(r->origin, route.path[i + 1])) break;
							if (inst.isForbiddenArc(route.path[j], r->destination))
								continue;
						}
						
						if (route.isInsertionCapacityFeasible(r, i, j) 
							&& route.isInsertionTimeFeasible(r, i, j)) {
								timeAndCapacityFeasibleFound = true;
								if (route.isInsertionBatteryFeasible(r, i, j)) {
									batset.emplace_back(v, i, j, nullptr, nullptr);
									batteryNotFound = false;
								}
						}
						
					}
				}
				if (batteryNotFound && timeAndCapacityFeasibleFound  && includeCS) {

					bool positionFound = false;
					std::vector<Node*> zero_load_nodes;
					for (int node_index = route.path.size() - 2; node_index != -1; node_index--)
						if (!route.loads[node_index]) zero_load_nodes.push_back(route.path[node_index]);
					for (Node* node:zero_load_nodes)
					{
						    int node_index = route.node_indices[node];
							CStation* min_s = 
								route.findBestChargingStationAfter(node_index,s.stationVisits);
							if (min_s != nullptr &&
								route.getAddedDistance(inst.nodes[min_s->id - 1],node_index) <= route.FTS[node_index + 1])
							{
								Node* cs_node = inst.nodes[min_s->id - 1];
								route.insertNode(cs_node, node_index + 1);
								route.updateMetrics();
								route.BasicScheduling();
								if (route.isFeasible()) {
									length = route.path.size() - 1;
									for (size_t i = 0; i < length; i++)
									{

										if (inst.isForbiddenArc(route.path[i], r->origin)) continue;

										for (size_t j = i; j < length; j++)
										{
											if (inst.isForbiddenArc(r->destination, route.path[j + 1]))
												continue;

											if (i != j) {
												if (inst.isForbiddenArc(r->origin, route.path[i + 1])) break;
												if (inst.isForbiddenArc(route.path[j], r->destination))
													continue;
											}
											if (route.isInsertionCapacityFeasible(r, i, j) &&
												route.isInsertionTimeFeasible(r, i, j) &&
												route.isInsertionBatteryFeasible(r, i, j))
											{
												batset.emplace_back(v, i, j,node,min_s);
												positionFound = true;
											}
										}
									}
								}

								
								route.removeNode(route.node_indices[cs_node]);
								route.updateMetrics();
								if (positionFound) break;
							}

					}
				}
			}
			
			return batset;
		}

		Solution Init1() {
			Solution solution;	
			std::random_device rd;
			RandLib randlib(std::move(rd()));
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
					if (min_pos.chargingStation != nullptr)
						inserted_route.insertNode(inst.nodes[min_pos.chargingStation->id - 1],inserted_route.node_indices[min_pos.node_before_station]+1);
					inserted_route.insertRequest(request, min_pos.origin_pos + 1, min_pos.dest_pos + 1);
					inserted_route.updateMetrics(true);
					solution.addRoute(inserted_route);
					solution.vehicleOfRequest.try_emplace(request,min_pos.vehicle);
				}
				else solution.rejected.push_back(request); 
			}
			return solution;
		}

#pragma region Destroy Operators

		void WorstRemoval(Solution& s, const std::array<double, 2>& arguments)
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
				[](const std::pair<Request*, std::pair<EAV*, double>>& pair1, const std::pair<Request*, std::pair<EAV*, double>>& pair2)
				{ return pair1.second.second > pair2.second.second; }
			);
			for (size_t i = 0; i < removal_number; i++) {
				int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * requestPosition.size());
				Route route = s.routes[requestPosition[index].second.first];
				route.removeRequest(requestPosition[index].first);
				route.updateMetrics(true);
				s.removed.push_back(requestPosition[index].first);
				s.addRoute(route);
				s.vehicleOfRequest.erase(requestPosition[index].first);
				requestPosition.erase(requestPosition.begin() + index);
			}
		}

		void RandomRemoval(Solution& s, const std::array<double, 2>& arguments)
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
					s.removed.push_back(req);
					s.addRoute(route);
					s.vehicleOfRequest.erase(req);
					i++;
				}
			}
		}

		void ZeroSplitRemoval(Solution& s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (!removal_number) return;
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
				[](const std::pair<std::vector<Request*>, std::pair<EAV*, double>>& pair1, const std::pair<std::vector<Request*>, std::pair<EAV*, double>>& pair2)
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
					s.vehicleOfRequest.erase(req);
					i++;
				}

				route.updateMetrics(true);
				s.addRoute(route);
				zssPosition.erase(zssPosition.begin() + index);
			}
		}

		void EntireRouteRemoval(Solution& s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (!removal_number) return;
			int i = 0;
			while (i < removal_number) {
				EAV* vehicle = inst.vehicles[randlib.randint(0, inst.vehicles.size()-1)];
				if (!s.routes[vehicle].hasNoRequests()) {
					for (Request* req : s.routes[vehicle].requests) {
						s.removed.push_back(req);
						s.vehicleOfRequest.erase(req);
					}
					i += s.routes[vehicle].requests.size();

					Route route(vehicle);
					route.insertNode(inst.getDepot(vehicle, "start"), 0);
					route.insertNode(inst.getDepot(vehicle, "end"), 1);
					route.updateMetrics(true);
					s.addRoute(route);
				}
			}

		}

		void SimilarityRemoval(Solution& s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (removal_number)
			{
				Request* removed_req;
				if (s.rejected.empty()) {
					removed_req = inst.requests[randlib.randint(0, inst.requests.size() - 1)];
					s.removed.push_back(removed_req);
					Route random_route = s.routes[s.vehicleOfRequest[removed_req]];
					s.vehicleOfRequest.erase(removed_req);
					random_route.removeRequest(removed_req);
					random_route.updateMetrics(true);
					s.addRoute(random_route);
					removal_number--;
				}
				else removed_req = s.rejected[randlib.randint(0, s.rejected.size() - 1)];
				
				std::vector<Request*> similarRequests = inst.similarRequestsSorted[removed_req->origin->id - 1];
				auto it = std::remove_if(similarRequests.begin(), similarRequests.end(), [&](Request* r) {
					return !s.vehicleOfRequest.contains(r);
				});
				similarRequests.erase(it,similarRequests.end());
				size_t i = 0;
				while (i < removal_number) {
					int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * similarRequests.size());
					Request* selectedRequest = similarRequests[index];
					if (s.vehicleOfRequest.contains(selectedRequest)) {
						Route route = s.routes[s.vehicleOfRequest[selectedRequest]];
						route.removeRequest(selectedRequest);
						route.updateMetrics(true);
						similarRequests.erase(similarRequests.begin() + index);
						s.addRoute(route);
						s.removed.push_back(selectedRequest);
						s.vehicleOfRequest.erase(selectedRequest);
						i++;
					}
					
				}
			}
			
			
		}

#pragma endregion 

#pragma region Repair Operators

		void GreedyInsertion(Solution& s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			while (!s.removed.empty()) {
				Position bestMove;
				double bestCost(DBL_MAX) ;
				Request* minUser=nullptr;
				for (Request* request : s.removed) {
					std::vector<Position> possibleMoves = InsertionNeighborhood(request, s, inst.vehicles,true);
					if (!possibleMoves.empty()) {
						Position min_pos = *std::min_element(possibleMoves.begin(), possibleMoves.end(),
							[&s, request](const Position& p1, const Position& p2) {return s.getInsertionCost(request, p1) < s.getInsertionCost(request, p2); }
						);

						if (s.getInsertionCost(request, min_pos) != DBL_MAX) {
							bestMove = std::move(min_pos);
							minUser = request;
						}
					}
				}
				if (bestCost == DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.resize(0);
				}
				else {
					Route new_route = s.routes[bestMove.vehicle];
					if (bestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1],new_route.node_indices[bestMove.node_before_station]+1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.vehicleOfRequest.try_emplace(minUser,bestMove.vehicle);
					s.addRoute(new_route);
				}
			}
			
		}

		void RandomInsertion(Solution& s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			for (int i = 0; i < s.removed.size(); i++) {
				int index = randlib.randint(0, s.removed.size() - 1);
				Request* request = s.removed[index];
				std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, inst.vehicles, true);
				auto it = std::remove_if(feasibleMoves.begin(), feasibleMoves.end(),
					[&s, request](const Position& move) {return s.getInsertionCost(request, move) == DBL_MAX; }
				);
				feasibleMoves.erase(it, feasibleMoves.end());
				if (feasibleMoves.size()) {
					Position bestMove = feasibleMoves[randlib.randint(0,feasibleMoves.size()-1)];
					Route new_route = s.routes[bestMove.vehicle];
					if (bestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1], new_route.node_indices[bestMove.node_before_station]+1);
					new_route.insertRequest(request, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(s.removed.begin() + index);
					s.vehicleOfRequest.try_emplace(request,bestMove.vehicle);
					s.addRoute(new_route);
				}
			}
			rejectedReinsertion(s);
			s.removed.resize(0);
		}

		void RegretInsertion(Solution& s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			int kappa = randlib.randint(2,inst.vehicles.size());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			while (!s.removed.empty()) {
				Position bestMove;
				bestMove.cost = DBL_MAX;
				double max_regret = -DBL_MAX;
				Request* minUser=nullptr;
				for (Request* request : s.removed) {
					std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, inst.vehicles,true);
					if (feasibleMoves.empty()) continue;
					std::for_each(feasibleMoves.begin(), feasibleMoves.end(), [&](Position& p) {
						p.cost = s.getInsertionCost(request, p);
					});
					std::vector<Position> RouteBestMove;
					for (EAV* v : inst.vehicles)
						RouteBestMove.push_back(
							*std::min_element(feasibleMoves.begin(), feasibleMoves.end(), [v](const Position& pos1, const Position& pos2)
								{	
									if (v->id != pos1.vehicle->id) return false;
									if (v->id != pos2.vehicle->id) return true;
									return pos1.cost < pos2.cost;
							    }
							)
						);
					

					std::sort(RouteBestMove.begin(), RouteBestMove.end(), [](const Position& p1, const Position& p2) 
					{return p1.cost < p2.cost;});


					double regret(0.0);
					int counter = 1;
					for (auto vehicleMovePair : RouteBestMove) {
						if (counter > kappa) break;
						regret+=vehicleMovePair.cost - RouteBestMove[0].cost;
						counter++;
					}
					if (max_regret < regret) {
						bestMove = RouteBestMove[0];
						max_regret = regret;
						minUser = request;
					}
					else if (regret == max_regret && RouteBestMove[0].cost < bestMove.cost) {
							bestMove = RouteBestMove[0];
							minUser = request;
					}
					
				}
				if (bestMove.cost==DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.resize(0);
				}
				else {
					Route new_route = s.routes[bestMove.vehicle];
					if (bestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1], new_route.node_indices[bestMove.node_before_station] + 1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.addRoute(new_route);
					s.vehicleOfRequest.try_emplace(minUser,bestMove.vehicle);
				}

			}
		}

#pragma endregion

		void rejectedReinsertion(Solution& s)
		{

			for (Request* removedRequest:s.removed) {
				bool insertionFailed = true;
				for (Request* req:inst.similarRequestsSorted[removedRequest->origin->id-1]) {
					if (!s.vehicleOfRequest.contains(req)) continue;
					Route route = s.routes[s.vehicleOfRequest[req]];
					std::pair<size_t, size_t> position = route.getRequestPosition(req);
					route.removeRequest(req);
					route.insertRequest(removedRequest, position.first, position.second - 1);
					route.updateMetrics();
					if (route.isFeasible()) {
						Solution intermediate = s;
						intermediate.vehicleOfRequest.erase(req);
						intermediate.vehicleOfRequest.try_emplace(removedRequest,route.vehicle);
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
							if (bestMove.chargingStation != nullptr)
								new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1], new_route.node_indices[bestMove.node_before_station] + 1);
							new_route.insertRequest(req, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
							new_route.updateMetrics(true);
							intermediate.addRoute(new_route);
							intermediate.vehicleOfRequest.try_emplace(req,bestMove.vehicle);
							s = std::move(intermediate);
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



