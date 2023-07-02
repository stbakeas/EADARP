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

	bool SimulatedAnnealingAcceptanceCriterion(const Solution& candidate, const Solution& incumbent, double current_temperature, RandLib& randlib) {
		return randlib.rand() < std::exp((incumbent.objectiveValue() - candidate.objectiveValue()) / current_temperature);
	}

	int RouletteWheelSelection(const std::vector<Statistics>& stats, double weightSum, RandLib& randlib) {
		double randomNumber = randlib.rand();
		double offset(0.0);
		for (int i = 0; i < stats.size(); i++) {
			offset += stats[i].weight / weightSum;
			if (randomNumber < offset) return i;
		}
	}

	Run ALNS(const Solution& initial,std::array<std::vector<Statistics>, 2> stats, unsigned int max_iterations, double temperature_control, double removalRandomness, double removalPercentage, int segment_size, double reaction_factor)
	{
		double temperature, cooling_rate(1.0);
		std::array<double, 2> weightSum;
		for (auto& rOperator : stats[0]) weightSum[0] += rOperator.weight;
		for (auto& iOperator : stats[1]) weightSum[1] += iOperator.weight;
		std::random_device rd;
		RandLib randlib(rd());
		std::array<double, 2> removalArguments = { removalPercentage,removalRandomness };
		std::array<double, 2> insertionArguments;
		Run run;
		auto start = std::chrono::steady_clock::now();
		Solution incumbent = run.best = run.init = initial;
		temperature = -temperature_control * run.best.objectiveValue() / log(0.5);
		cooling_rate = pow(0.01 / temperature, 1.0 / (max_iterations));
		Solution s;
		std::vector<EAV*> allVehicles = inst.vehicles;
		for (unsigned int iter = 1; iter <= max_iterations; ++iter) {
			int removalIndex = RouletteWheelSelection(stats[0], weightSum[0], randlib);
			int insertionIndex = RouletteWheelSelection(stats[1], weightSum[1], randlib);
			s = incumbent;
			//Removal step. Includes removing redundant charging stations
			stats[0][removalIndex].heuristic(s, removalArguments);
			stats[0][removalIndex].attempts++;
			

			for (EAV* v : inst.vehicles) {
				Route route = s.routes[v];
				if (route.deleteRedundantChargingStations()) {
					route.updateMetrics();
					s.addRoute(route);
				}
			}

			//Insertion step
			stats[1][insertionIndex].heuristic(s, insertionArguments);
			stats[1][insertionIndex].attempts++;

			//Select returning depots
			std::unordered_set<Node*> visitedDepots;
			std::shuffle(allVehicles.begin(), allVehicles.end(), rd);
			for (EAV* v :allVehicles) {
				Node* nodeBeforeEnd = *(s.routes[v].path.end() - 2);
				for (Node* depot : inst.closestDestinationDepot[nodeBeforeEnd]) {
					if (!visitedDepots.contains(depot)) {
						Route route = s.routes[v];
						route.travel_distance -= inst.getTravelTime(nodeBeforeEnd, route.path.back());
						route.node_indices.erase(route.path.back());
						route.path.back() = depot;
						route.travel_distance += inst.getTravelTime(nodeBeforeEnd,depot);
						route.node_indices.emplace(depot, route.path.size() - 1);
						route.updateMetrics();
						if (route.isFeasible()) {
							s.addRoute(route);
							visitedDepots.insert(depot);
							break;
						}
					}
				}
			}

			if (dbl_round(s.objectiveValue(), 2) < dbl_round(incumbent.objectiveValue(), 2)) {
				if (dbl_round(s.objectiveValue(), 2) < dbl_round(run.best.objectiveValue(), 2)) {
					run.best = incumbent= std::move(s);
					stats[0][removalIndex].score += 10;
					stats[1][insertionIndex].score += 10;
					run.best_iter = iter;
					printf("New best Found:%f, Rejected: %d\n", s.objectiveValue(), s.rejected.size());
				}
				else {
					incumbent = std::move(s);
					stats[0][removalIndex].score += 7;
					stats[1][insertionIndex].score += 7;
				}
			}
			else if (SimulatedAnnealingAcceptanceCriterion(s, incumbent, temperature, randlib)) {
				incumbent = std::move(s);
				stats[0][removalIndex].score += 5;
				stats[1][insertionIndex].score += 5;

			}


			

			temperature *= cooling_rate;

			if (iter % segment_size == 0) {

				for (int i = 0; i < 2; i++) {
					weightSum[i] = 0.0;
					for (auto it = stats[i].begin(); it != stats[i].end(); ++it) {
						if (it->attempts) {
							it->weight = (1.0 - reaction_factor) * it->weight + reaction_factor * it->score / it->attempts;
							it->attempts = 0;
						}
						weightSum[i] += it->weight;
						it->score = 0;
					}
				}
			}

			run.elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() / 1000.0;
		}
		//printf("Elapsed time: %f\n", run.elapsed_seconds);
		return run;
	}

	namespace details {

		std::vector<Position> InsertionNeighborhood(const Request* r, Solution& s, const std::vector<EAV*>& available_vehicles, bool includeCS)
		{
			std::vector<Position> batset;
			batset.reserve(inst.vehicles.size() * 2 * inst.requests.size());
			for (EAV* v : available_vehicles) {
				if (r->forbidden_vehicles.contains(v->id)) continue;
				Route& route = s.routes[v];
				size_t length = route.path.size() - 1;
				bool batteryNotFound = true, timeAndCapacityFound = false;

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

						if (route.isInsertionCapacityFeasible(r, i, j)
							&& route.isInsertionTimeFeasible(r, i, j)) {
							timeAndCapacityFound = true;
							if (route.isInsertionBatteryFeasible(r, i, j)) {
								batset.emplace_back(v, i, j, nullptr, nullptr);
								batteryNotFound = false;
							}
						}

					}
				}
				if (batteryNotFound && timeAndCapacityFound && includeCS) {

					bool positionFound = false;
					for (int node_index = route.path.size() - 2; node_index != -1; node_index--) {
						if (route.loads[node_index]) continue;
						CStation* min_s =
							route.findBestChargingStationAfter(node_index, s.stationVisits);
						if (min_s == nullptr ||
							route.getAddedDistance(inst.nodes[min_s->id - 1], node_index) > route.FTS[node_index + 1])
							continue;
						Node* node = route.path[node_index];
						Node* cs_node = inst.nodes[min_s->id - 1];
						route.insertNode(cs_node, node_index + 1);
						route.updateMetrics();
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

										batset.emplace_back(v, i, j, node, min_s);
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

			return batset;
		}

		Solution Init1() {
			Solution solution;
			std::random_device rd;
			RandLib randlib(rd());
			solution.AddDepots();
			std::vector<Request*> unassigned = inst.requests;
			std::sort(unassigned.begin(), unassigned.end(), [](Request* r1, Request* r2) {return r1->origin->earliest < r2->origin->earliest; });
			for (Request* request : unassigned) {
				std::vector<Position> possibleMoves = InsertionNeighborhood(request, solution, inst.vehicles, true);
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
					Route inserted_route;
					inserted_route.PartialCopy(solution.routes[min_pos.vehicle]);
					if (min_pos.chargingStation != nullptr)
						inserted_route.insertNode(inst.nodes[min_pos.chargingStation->id - 1], inserted_route.node_indices[min_pos.node_before_station] + 1);
					inserted_route.insertRequest(request, min_pos.origin_pos + 1, min_pos.dest_pos + 1);
					inserted_route.updateMetrics(true);
					solution.addRoute(inserted_route);
					solution.vehicleOfRequest.emplace(request, min_pos.vehicle);
				}
				else solution.rejected.push_back(request);
			}
			return solution;
		}

#pragma region Destroy Operators

		void WorstRemoval(Solution& s, const std::array<double, 2>& arguments)
		{

			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);

			std::vector<std::pair<Request*, double>> requestPosition;
			requestPosition.reserve(num_of_assigned_requests);
			for (EAV* v : inst.vehicles) {

				for (Request* r : s.routes[v].requests)
				{
					Route new_route;
					new_route.PartialCopy(s.routes[v]);
					new_route.removeRequest(r);
					new_route.updateMetrics();
					requestPosition.emplace_back(r, 0.75 * (s.routes[v].travel_distance - new_route.travel_distance) +
						0.25 * (s.routes[v].excess_ride_time - new_route.excess_ride_time));
				}
			}
			std::sort(requestPosition.begin(), requestPosition.end(),
				[](const std::pair<Request*, double>& pair1, const std::pair<Request*, double>& pair2)
				{ return pair1.second > pair2.second; }
			);

			std::random_device rd;
			RandLib randlib(rd());
			std::unordered_map<EAV*, std::vector<Request*>> modifications;
			for (size_t i = 0; i < removal_number; i++) {
				int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * requestPosition.size());
				EAV* vehicle = s.vehicleOfRequest[requestPosition[index].first];
				s.vehicleOfRequest.erase(requestPosition[index].first);
				s.removed.push_back(requestPosition[index].first);
				modifications[vehicle].push_back(requestPosition[index].first);
				requestPosition.erase(requestPosition.begin() + index);
			}
			for (const auto& [vehicle, requests] : modifications) {
				Route route;
				route.PartialCopy(s.routes[vehicle]);
				for (Request* req : requests) route.removeRequest(req); 
				route.updateMetrics(true);
				s.addRoute(route);
			}
		}

		void RandomRemoval(Solution& s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);

			int i = 0;
			std::unordered_map <EAV*, std::vector<Request*>> modifications;
		    while (i < removal_number) {
				EAV* random_vehicle = inst.vehicles[randlib.randint(0, inst.vehicles.size() - 1)];
				Route& route = s.routes[random_vehicle];
				if (!route.hasNoRequests()) {
					int index = randlib.randint(0, route.requests.size() - 1);
					Request* req = route.requests[index];
					route.requests.erase(std::remove(route.requests.begin(), route.requests.end(), req), route.requests.end());
					s.removed.push_back(req);
					s.vehicleOfRequest.erase(req);
					
					modifications[random_vehicle].push_back(req);
					i++;
				}
			}
			for (const auto& [vehicle, requests] : modifications) {
				Route route;
				route.PartialCopy(s.routes[vehicle]);
				for (Request* req : requests) {
					route.removeNode(route.node_indices[req->origin]);
					route.removeNode(route.node_indices[req->destination]);
				}
				route.updateMetrics(true);
				s.addRoute(route);
			}
		}

		void ZeroSplitRemoval(Solution& s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			size_t num_of_assigned_requests = inst.requests.size() - s.rejected.size();
			int removal_number = ceil(num_of_assigned_requests * arguments[0]);
			if (!removal_number) return;
			std::vector<std::pair<std::vector<Request*>, double>> zssPosition;
			std::vector<Request*> removedRequests;
			zssPosition.reserve(inst.requests.size());
			for (EAV* v : inst.vehicles) {
				for (const auto& pair : s.routes[v].natural_sequences) {
					removedRequests.reserve((pair.second - pair.first) / 2);
					double cost(0.0);
					for (int i = pair.first; i < pair.second; i++) {
						if (s.routes[v].path[i]->isOrigin()) removedRequests.push_back(inst.getRequest(s.routes[v].path[i]));
						cost += inst.getTravelTime(s.routes[v].path[i], s.routes[v].path[i + 1]) + s.routes[v].late_waiting_times[i];
					}
					zssPosition.emplace_back(removedRequests, cost);
					removedRequests.clear();
				}
			}

			std::sort(zssPosition.begin(), zssPosition.end(),
				[](const std::pair<std::vector<Request*>, double>& pair1, const std::pair<std::vector<Request*>, double>& pair2)
				{ return pair1.second > pair2.second; }
			);


			int i = 0;
			std::unordered_map<EAV*, std::vector<Request*>> modifications;
			while (i < removal_number) {
				int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * zssPosition.size());
				EAV* vehicle = s.vehicleOfRequest[zssPosition[index].first[0]];
				for (Request* req : zssPosition[index].first) {
					modifications[vehicle].push_back(req);
					s.vehicleOfRequest.erase(req);
					s.removed.push_back(req);
					i++;
				}
				zssPosition.erase(zssPosition.begin() + index);
			}
			for (const auto& [vehicle, requests] : modifications) {
				Route route;
				route.PartialCopy(s.routes[vehicle]);
				for (Request* req : requests) route.removeRequest(req);
				route.updateMetrics(true);
				s.addRoute(route);
				
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
					EAV* vehicle = s.vehicleOfRequest[removed_req];
					Route random_route;
					random_route.PartialCopy(s.routes[vehicle]);
					s.vehicleOfRequest.erase(removed_req);
					random_route.removeRequest(removed_req);
					random_route.updateMetrics(true);
					s.addRoute(random_route);
					removal_number--;
				}
				else removed_req = s.rejected[randlib.randint(0, s.rejected.size() - 1)];

				std::vector<Request*> similarRequests;
				similarRequests.reserve(inst.requests.size() - 1);
				std::copy_if(inst.similarRequestsSorted[removed_req->origin->id - 1].begin(),
					inst.similarRequestsSorted[removed_req->origin->id - 1].end(),
					std::back_inserter(similarRequests),
					[&s](Request* r) {return s.vehicleOfRequest.contains(r); });
				size_t i = 0;
				while (i < removal_number) {
					int index = floor(pow(randlib.unifrnd(0, 0.99), arguments[1]) * similarRequests.size());
					Request* selectedRequest = similarRequests[index];
					if (s.vehicleOfRequest.contains(selectedRequest)) {
						EAV* vehicle = s.vehicleOfRequest[selectedRequest];
						Route route;
						route.PartialCopy(s.routes[vehicle]);
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
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			std::vector<Position> possibleMoves;
			while (!s.removed.empty()) {
				Position bestMove;
				Request* minUser = nullptr;
				for (Request* request : s.removed) {
					possibleMoves = InsertionNeighborhood(request, s, inst.vehicles, true);
					Position min_pos;
					for (auto& p : possibleMoves) {
						p.cost = s.getInsertionCost(request, p);
						if (p.cost < min_pos.cost)
							min_pos = std::move(p);
					}
					if (min_pos.cost < bestMove.cost) {
						bestMove = std::move(min_pos);
						minUser = request;
					}

				}
				if (bestMove.cost == DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.resize(0);
				}
				else {
					Route new_route;
					new_route.PartialCopy(s.routes[bestMove.vehicle]);
					if (bestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1], new_route.node_indices[bestMove.node_before_station] + 1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.vehicleOfRequest.emplace(minUser, bestMove.vehicle);
					s.addRoute(new_route);
				}
			}

		}

		void RandomInsertion(Solution& s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			std::vector<Position> feasibleMoves;
			std::vector<EAV*> allVehicles = inst.vehicles;
			for (int i = 0; i < s.removed.size(); i++) {
				int index = randlib.randint(0, s.removed.size() - 1);
				Request* request = s.removed[index];
				std::shuffle(allVehicles.begin(), allVehicles.end(), rd);
				std::vector<Position>::iterator bestMove=feasibleMoves.end();
				for (EAV* v : allVehicles) {
					feasibleMoves = InsertionNeighborhood(request, s, { v }, true);
					std::shuffle(feasibleMoves.begin(), feasibleMoves.end(), rd);
					bestMove = std::ranges::find_if(feasibleMoves, [&s, request](const Position& p) {
						return s.getInsertionCost(request, p) != DBL_MAX;
					});
					if (bestMove != feasibleMoves.end()) break;
				}

				if (bestMove != feasibleMoves.end()) {
					Route new_route;
					new_route.PartialCopy(s.routes[bestMove->vehicle]);
					if (bestMove->chargingStation != nullptr)
						new_route.insertNode(inst.nodes[bestMove->chargingStation->id - 1], new_route.node_indices[bestMove->node_before_station] + 1);
					new_route.insertRequest(request, bestMove->origin_pos + 1, bestMove->dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(s.removed.begin() + index);
					s.vehicleOfRequest.emplace(request, bestMove->vehicle);
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
			int kappa = randlib.randint(2, inst.vehicles.size());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			std::vector<Position> RouteBestMove;
			RouteBestMove.reserve(inst.vehicles.size());
			Position p;
			while (!s.removed.empty()) {
				Position bestMove;
				double max_regret = -DBL_MAX;
				Request* minUser = nullptr;
				for (Request* request : s.removed) {
					for (EAV* v : inst.vehicles) {
						std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, { v }, true);
						if (feasibleMoves.empty()) {
							RouteBestMove.push_back(p);
							continue;
						}
						int bestMoveIndex = 0;
						double minCost = s.getInsertionCost(request, feasibleMoves[0]);
						for (int i = 1; i < feasibleMoves.size(); i++) {
							double cost = s.getInsertionCost(request, feasibleMoves[i]);
							if (cost < minCost) {
								bestMoveIndex = i;
								minCost = cost;
							}

						}
						feasibleMoves[bestMoveIndex].cost = minCost;
						RouteBestMove.push_back(std::move(feasibleMoves[bestMoveIndex]));
					}


					std::sort(RouteBestMove.begin(), RouteBestMove.end(), [](const Position& p1, const Position& p2)
						{return p1.cost < p2.cost; });


					double regret = 0.0;
					int counter = 1;
					for (const auto& vehicleMovePair : RouteBestMove) {
						if (counter > kappa) break;
						regret += vehicleMovePair.cost - RouteBestMove[0].cost;
						counter++;
					}

					if (max_regret < regret) {
						bestMove = std::move(RouteBestMove[0]);
						max_regret = regret;
						minUser = request;
					}
					else if (regret == max_regret && RouteBestMove[0].cost < bestMove.cost) {
						bestMove = std::move(RouteBestMove[0]);
						minUser = request;
					}
					RouteBestMove.resize(0);
				}
				if (bestMove.cost == DBL_MAX) {
					std::move(s.removed.begin(), s.removed.end(), std::back_inserter(s.rejected));
					s.removed.resize(0);
				}
				else {
					Route new_route;
					new_route.PartialCopy(s.routes[bestMove.vehicle]);
					if (bestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1], new_route.node_indices[bestMove.node_before_station] + 1);
					new_route.insertRequest(minUser, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.addRoute(new_route);
					s.vehicleOfRequest.emplace(minUser, bestMove.vehicle);
				}

			}
		}

		void CachedPositionRegretInsertion(Solution& s, const std::array<double, 2>& arguments)
		{
			std::random_device rd;
			RandLib randlib(rd());
			size_t kappa = randlib.randint(2,inst.vehicles.size());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			std::unordered_map<Request*, std::vector<Position>> kBestMovesFor;
			kBestMovesFor.reserve(s.removed.size());
			for (Request* req : s.removed) {
				std::vector<Position> kBestMoves(kappa);
				std::vector<Position> feasibleMoves = InsertionNeighborhood(req, s,inst.vehicles, true);
				for (auto& insertionMove : feasibleMoves) insertionMove.cost = s.getInsertionCost(req, insertionMove);
				std::sort(feasibleMoves.begin(), feasibleMoves.end(), [](const Position& p1, const Position& p2)
					{return p1.cost < p2.cost; }
				);
				for (int i = 0; i < std::min(kappa, feasibleMoves.size()); i++)
					kBestMoves[i] = std::move(feasibleMoves[i]);

			}
			while (!s.removed.empty()) {
				double maxRegret = -DBL_MAX;
				Position globalBestMove;
				Request* bestUser = nullptr;
				for (auto& [request, moves] :kBestMovesFor) {
					double regret = 0.0;
					for (const auto& vehicleMove : moves)
						regret += vehicleMove.cost - moves[0].cost;
						
					if (maxRegret < regret) {
						maxRegret = regret;
						bestUser = request;
						globalBestMove = moves[0];
					}
					else if (regret == maxRegret && moves[0].cost < globalBestMove.cost) {
						globalBestMove = moves[0];
						bestUser = request;
					}
				}
				if (globalBestMove.cost == DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.resize(0);
				}

				else {
					Route new_route;
					new_route.PartialCopy(s.routes[globalBestMove.vehicle]);
					if (globalBestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[globalBestMove.chargingStation->id - 1], new_route.node_indices[globalBestMove.node_before_station] + 1);
					new_route.insertRequest(bestUser, globalBestMove.origin_pos + 1, globalBestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), bestUser), s.removed.end());
					s.vehicleOfRequest.emplace(bestUser, globalBestMove.vehicle);
					s.addRoute(new_route);
					kBestMovesFor.erase(bestUser);
					for (auto& [request, moves] : kBestMovesFor) {
						for (auto& pos : moves) {
							if (pos.vehicle->id == globalBestMove.vehicle->id ||
								s.stationVisits[pos.chargingStation]+1>inst.maxVisitsPerStation) {
								moves.clear();
								moves.resize(kappa);
								std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, inst.vehicles, true);
								for (auto& insertionMove : feasibleMoves)
									insertionMove.cost = s.getInsertionCost(request, insertionMove);
								std::sort(feasibleMoves.begin(), feasibleMoves.end(), [](const Position& p1, const Position& p2)
									{return p1.cost < p2.cost; }
								);
								for (int i = 0; i < std::min(kappa, feasibleMoves.size()); i++)
									moves[i] = std::move(feasibleMoves[i]);
								break;
							}

						}
						
					}
				}
			}
		}

		void CachedRegretInsertion(Solution& s, const std::array<double, 2>& arguments) {
			std::random_device rd;
			RandLib randlib(rd());
			int kappa = randlib.randint(2, inst.vehicles.size());
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			std::unordered_map<Request*, std::vector<Position>> vehicleBestMoveFor;
			vehicleBestMoveFor.reserve(s.removed.size());
			for (Request* req : s.removed) {
				std::vector<Position> RouteBestMove;
				RouteBestMove.reserve(inst.vehicles.size());
				for (EAV* v : inst.vehicles) {
					Position vehicleBestMove;
					vehicleBestMove.vehicle = v;
					std::vector<Position> feasibleMoves = InsertionNeighborhood(req, s, { v }, true);
					for (auto& insertionMove : feasibleMoves) {
						insertionMove.cost = s.getInsertionCost(req, insertionMove);
						if (insertionMove.cost < vehicleBestMove.cost )
							vehicleBestMove = std::move(insertionMove);
					}
					RouteBestMove.push_back(vehicleBestMove);
				}
				vehicleBestMoveFor.try_emplace(req, RouteBestMove);
			}
			while (!s.removed.empty()) {
				double maxRegret = -DBL_MAX;
				Position globalBestMove;
				Request* bestUser = nullptr;
				for (auto& [request, moves] : vehicleBestMoveFor) {
					std::sort(moves.begin(), moves.end(), [](const Position& p1, const Position& p2) {return p1.cost < p2.cost; });
					double regret = 0.0;
					int counter = 0;
					for (const auto& vehicleMove : moves) {
						if (counter > kappa) break;
						regret += vehicleMove.cost - moves[0].cost;
						counter++;
					}

					if (maxRegret < regret) {
						maxRegret = regret;
						bestUser = request;
						globalBestMove = moves[0];
					}
					else if (regret == maxRegret && moves[0].cost < globalBestMove.cost) {
						globalBestMove = moves[0];
						bestUser = request;
					}
				}
				if (globalBestMove.cost == DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.resize(0);
				}
			
				else {
					Route new_route;
					new_route.PartialCopy(s.routes[globalBestMove.vehicle]);
					if (globalBestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[globalBestMove.chargingStation->id - 1], new_route.node_indices[globalBestMove.node_before_station] + 1);
					new_route.insertRequest(bestUser, globalBestMove.origin_pos + 1, globalBestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), bestUser), s.removed.end());
					s.vehicleOfRequest.emplace(bestUser, globalBestMove.vehicle);
					s.addRoute(new_route);
					vehicleBestMoveFor.erase(bestUser);
					for (auto& [request, moves] : vehicleBestMoveFor) {
						Position newBestMoveForVehicle;
						newBestMoveForVehicle.vehicle = globalBestMove.vehicle;
						std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, { globalBestMove.vehicle }, true);
						for (auto& insertionMove : feasibleMoves) {
							insertionMove.cost = s.getInsertionCost(request, insertionMove);
							if (insertionMove.cost < newBestMoveForVehicle.cost)
								newBestMoveForVehicle = std::move(insertionMove);
						}
						auto iter = std::find_if(moves.begin(), moves.end(),
							[&globalBestMove](const Position& p)
							{return globalBestMove.vehicle->id == p.vehicle->id; });
						moves[std::distance(moves.begin(), iter)] = std::move(newBestMoveForVehicle);
					}

					for (auto& [request, moves] : vehicleBestMoveFor) {
						for (Position& p : moves) {
							if (p.chargingStation!=nullptr && s.stationVisits[p.chargingStation] + 1 > inst.maxVisitsPerStation) {
								std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, { p.vehicle }, true);
								Position newMove;
								newMove.vehicle = p.vehicle;
								for (auto& insertionMove : feasibleMoves) {
									insertionMove.cost = s.getInsertionCost(request, insertionMove);
									if (insertionMove.cost < newMove.cost)
										newMove = std::move(insertionMove);
								}
								p = std::move(newMove);
							}
						}
					}
				}


			}
		}

		void CachedGreedyInsertion(Solution& s, const std::array<double, 2>& arguments) {
			std::move(s.rejected.begin(), s.rejected.end(), std::back_inserter(s.removed));
			s.rejected.resize(0);
			std::unordered_map<Request*, std::vector<Position>> vehicleBestMoveFor;
			vehicleBestMoveFor.reserve(s.removed.size());
			for (Request* req : s.removed) {
				std::vector<Position> RouteBestMove;
				RouteBestMove.reserve(inst.vehicles.size());
				for (EAV* v : inst.vehicles) {
					Position vehicleBestMove;
					vehicleBestMove.vehicle = v;
					std::vector<Position> feasibleMoves = InsertionNeighborhood(req, s, { v }, true);
					for (auto& insertionMove : feasibleMoves) {
						insertionMove.cost = s.getInsertionCost(req, insertionMove);
						if (insertionMove.cost < vehicleBestMove.cost) 
							vehicleBestMove = std::move(insertionMove);
					}
					RouteBestMove.push_back(vehicleBestMove);
				}
				vehicleBestMoveFor.try_emplace(req, RouteBestMove);
			}
			while (!s.removed.empty()) {
				Position globalBestMove;
				Request* minUser = nullptr;
				for (const auto& [request,moves] : vehicleBestMoveFor) {
					Position insertionMove = *std::min_element(moves.begin(),moves.end(),
						[](const Position& p1, const Position& p2) {return p1.cost < p2.cost; });
					if (insertionMove.cost < globalBestMove.cost) {
						globalBestMove = std::move(insertionMove);
						minUser = request;
					}
				}
				if (globalBestMove.cost == DBL_MAX) {
					rejectedReinsertion(s);
					s.removed.resize(0);
				}
				else {
					Route new_route;
					new_route.PartialCopy(s.routes[globalBestMove.vehicle]);
					if (globalBestMove.chargingStation != nullptr)
						new_route.insertNode(inst.nodes[globalBestMove.chargingStation->id - 1], new_route.node_indices[globalBestMove.node_before_station] + 1);
					new_route.insertRequest(minUser, globalBestMove.origin_pos + 1, globalBestMove.dest_pos + 1);
					new_route.updateMetrics(true);
					s.removed.erase(std::remove(s.removed.begin(), s.removed.end(), minUser), s.removed.end());
					s.vehicleOfRequest.emplace(minUser, globalBestMove.vehicle);
					s.addRoute(new_route);
					vehicleBestMoveFor.erase(minUser);
					for (auto& [request, moves] : vehicleBestMoveFor) {
						Position newBestMoveForVehicle;
						newBestMoveForVehicle.vehicle = globalBestMove.vehicle;
						std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, { globalBestMove.vehicle }, true);
						for (auto& insertionMove : feasibleMoves) {
							insertionMove.cost = s.getInsertionCost(request, insertionMove);
							if (insertionMove.cost < newBestMoveForVehicle.cost)
								newBestMoveForVehicle = std::move(insertionMove);
						}
						moves[globalBestMove.vehicle->id - 2 * inst.requests.size() - 3] = std::move(newBestMoveForVehicle);
					}
					for (auto& [request, moves] : vehicleBestMoveFor) {
						for (Position& p : moves) {
							if (p.chargingStation != nullptr && s.stationVisits[p.chargingStation] + 1 > inst.maxVisitsPerStation) {
								std::vector<Position> feasibleMoves = InsertionNeighborhood(request, s, { p.vehicle }, true);
								Position newMove;
								newMove.vehicle = p.vehicle;
								for (auto& insertionMove : feasibleMoves) {
									insertionMove.cost = s.getInsertionCost(request, insertionMove);
									if (insertionMove.cost < newMove.cost)
										newMove = std::move(insertionMove);
								}
								p = std::move(newMove);
							}
						}
					}
					
				}

			}

		}


#pragma endregion

		void rejectedReinsertion(Solution& s)
		{
			for (Request* removedRequest : s.removed) {
				bool insertionFailed = true;
				for (Request* req : inst.similarRequestsSorted[removedRequest->origin->id - 1]) {
					if (!s.vehicleOfRequest.contains(req)) continue;
					Route route;
					route.PartialCopy(s.routes[s.vehicleOfRequest[req]]);
					std::pair<size_t, size_t> position = route.getRequestPosition(req);
					route.removeRequest(req);
					route.insertRequest(removedRequest, position.first, position.second - 1);
					route.updateMetrics();
					if (route.isFeasible()) {
						Solution intermediate = s;
						intermediate.vehicleOfRequest.erase(req);
						intermediate.vehicleOfRequest.emplace(removedRequest, route.vehicle);
						intermediate.addRoute(route);
						std::vector<Position> possibleMoves = InsertionNeighborhood(req, intermediate, inst.vehicles, true);
						Position bestMove;
						for (auto& move : possibleMoves) {
							move.cost = intermediate.getInsertionCost(req, move);
							if (move.cost < bestMove.cost)
								bestMove = std::move(move);
						}
						if (bestMove.cost != DBL_MAX) {
							Route new_route;
							new_route.PartialCopy(intermediate.routes[bestMove.vehicle]);
							if (bestMove.chargingStation != nullptr)
								new_route.insertNode(inst.nodes[bestMove.chargingStation->id - 1], new_route.node_indices[bestMove.node_before_station] + 1);
							new_route.insertRequest(req, bestMove.origin_pos + 1, bestMove.dest_pos + 1);
							new_route.updateMetrics(true);
							intermediate.addRoute(new_route);
							intermediate.vehicleOfRequest.emplace(req, bestMove.vehicle);
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



