#include <iostream>
#include "Instance.h"
#include "Algorithms.h"
#include "Solution.h"
#include "Route.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include <chrono>
#include <thread>
#include "RandLib.h"

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <fstream>


using namespace std;
using namespace operations_research;






//struct DataModel {
//	std::vector < std::vector<int>> distance_matrix;
//	std::vector<std::pair<int, int>> time_windows;
//	std::vector <std::vector<RoutingIndexManager::NodeIndex>> requests;
//	std::vector<RoutingIndexManager::NodeIndex> start_depots;
//	std::vector<RoutingIndexManager::NodeIndex> end_depots;
//	std::vector<RoutingIndexManager::NodeIndex> charging_stations;
//	std::vector<int64_t> vehicle_capacities;
//	std::vector<int> maximum_ride_times;
//	int max_fuel = 180;
//};
//
//void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
//	const RoutingModel& routing, const Assignment& solution) {
//	const RoutingDimension& time_dimension = routing.GetDimensionOrDie("Time");
//	const RoutingDimension& fuel_dimension = routing.GetDimensionOrDie("Fuel");
//	int64_t total_time{ 0 };
//	for (int vehicle_id = 0; vehicle_id < inst.vehicles.size(); ++vehicle_id) {
//		int64_t index = routing.Start(vehicle_id);
//		LOG(INFO) << "Route for vehicle " << vehicle_id << ":";
//		std::ostringstream route;
//		while (routing.IsEnd(index) == false) {
//			auto time_var = time_dimension.CumulVar(index);
//			auto fuel_var = fuel_dimension.CumulVar(index);
//			route <<"Node: " << manager.IndexToNode(index).value() << " Start Of Service: " << solution.Value(time_var)
//				<< " Time Window("
//				<< data.time_windows.at(manager.IndexToNode(index).value()).first << ", " << data.time_windows.at(manager.IndexToNode(index).value()).second
//				<< ") " << "Fuel Left: " << solution.Value(fuel_var) << " -> ";
//			index = solution.Value(routing.NextVar(index));
//		}
//		auto time_var = time_dimension.CumulVar(index);
//		auto fuel_var = fuel_dimension.CumulVar(index);
//		LOG(INFO) << route.str() << "Node: " << manager.IndexToNode(index).value() << " Start Of Service: " << solution.Value(time_var)
//			<< " Time Window("
//			<< data.time_windows.at(manager.IndexToNode(index).value()).first << ", " << data.time_windows.at(manager.IndexToNode(index).value()).second
//			<< ") "<< "Fuel Left: " << solution.Value(fuel_var);
//		LOG(INFO) << "Time of the route: " << solution.Min(time_var) << "min";
//		total_time += solution.Min(time_var);
//	}
//	LOG(INFO) << "Total time of all routes: " << total_time << "min";
//	LOG(INFO) << "";
//	LOG(INFO) << "Advanced usage:";
//	LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
//}
//
//void HeterogeneousDARP() {
//	// Instantiate the data problem.
//	DataModel data;
//	data.distance_matrix = inst.distanceMatrix;
//	data.time_windows.resize(2 * inst.requests.size() + 2*inst.vehicles.size()+inst.charging_stations.size());
//	for (EAV* v : inst.vehicles) {
//		data.start_depots.push_back(RoutingIndexManager::NodeIndex{ inst.getDepot(v,"start")->id-1});
//		
//		data.end_depots.push_back(RoutingIndexManager::NodeIndex{ inst.getDepot(v,"end")->id-1 });
//		data.vehicle_capacities.push_back(v->capacity);
//		data.time_windows.at(inst.getDepot(v,"start")->id - 1) = {inst.getDepot(v,"start")->earliest,inst.getDepot(v,"start")->latest};
//		data.time_windows.at(inst.getDepot(v, "end")->id - 1) = { inst.getDepot(v,"end")->earliest,inst.getDepot(v,"end")->latest };
//	}
//	for (Request* r : inst.requests)
//	{
//		int pickup = r->origin->id;
//		int delivery = r->destination->id;
//		data.requests.push_back({ RoutingIndexManager::NodeIndex{pickup-1},RoutingIndexManager::NodeIndex{delivery-1} });
//		data.maximum_ride_times.push_back(r->destination->maximum_travel_time);
//		data.time_windows.at(pickup - 1) = { r->origin->earliest,r->origin->latest };
//		data.time_windows.at(delivery - 1) = { r->destination->earliest,r->destination->latest };
//	}
//	for (CStation* s:inst.charging_stations){
//		data.charging_stations.push_back(RoutingIndexManager::NodeIndex{s->id-1});
//		data.time_windows.at(s->id - 1) = {inst.nodes.at(s->id - 1)->earliest,inst.nodes.at(s->id - 1)->latest};
//	}
//
//	// Create Routing Index Manager
//	RoutingIndexManager manager(data.distance_matrix.size(), inst.vehicles.size(), data.start_depots, data.end_depots);
//
//	// Create Routing Model.
//	RoutingModel routing(manager);
//
//	// Create and register a transit callback.
//	const int transit_callback_index = routing.RegisterTransitCallback(
//		[&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
//			// Convert from routing variable Index to distance matrix NodeIndex.
//			auto from_node = manager.IndexToNode(from_index).value();
//			auto to_node = manager.IndexToNode(to_index).value();
//			return data.distance_matrix[from_node][to_node];
//		});
//
//	// Define cost of each arc.
//	routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
//
//	// Add Distance constraint.
//	routing.AddDimension(transit_callback_index,  // transit callback
//		0,                       // no slack
//		inst.Horizon,  // vehicle maximum travel distance
//		true,  // start cumul to zero
//		"Distance");
//	RoutingDimension* distance_dimension =
//		routing.GetMutableDimension("Distance");
//	distance_dimension->SetGlobalSpanCostCoefficient(100);
//	
//
//	const int demand_callback_index = routing.RegisterUnaryTransitCallback(
//		[&data, &manager](int64_t from_index) -> int64_t {
//			// Convert from routing variable Index to demand NodeIndex.
//			int from_node = manager.IndexToNode(from_index).value();
//			return inst.nodes.at(from_node)->load;
//		});
//	routing.AddDimensionWithVehicleCapacity(
//		demand_callback_index,    // transit callback index
//		int64_t{ 0 },               // null capacity slack
//		data.vehicle_capacities,  // vehicle maximum capacities
//		true,                     // start cumul to zero
//		"Capacity");
//
//	//Create and register a fuel callback.
//	const int fuel_callback_index = routing.RegisterTransitCallback(
//		[&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
//			// Convert from routing variable Index to distance matrix NodeIndex.
//			auto from_node = manager.IndexToNode(from_index).value();
//			auto to_node = manager.IndexToNode(to_index).value();
//			return -data.distance_matrix[from_node][to_node];
//		});
//
//	//Add Fuel Dimension. Used for the modeling of vehicle recharging.
//	routing.AddDimension(fuel_callback_index,
//		data.max_fuel,
//		data.max_fuel,
//		false,
//		"Fuel");
//	RoutingDimension* fuel_dimension = routing.GetMutableDimension("Fuel");
//
//	
//	RoutingModel::VariableIndexEvaluator2 fuel_dependent_time_evaluator = 
//		[&data,&manager](int64_t from_index,int64_t to_index) {
//		RoutingIndexManager::NodeIndex from_node = manager.IndexToNode(from_index);
//		RoutingIndexManager::NodeIndex to_node = manager.IndexToNode(to_index);
//		const std::function<int64_t(int64_t)> fuel_dependent_transit_time= [from_node, to_node,data](
//			int64_t current_battery) -> int64_t {
//				int64_t transit_time= data.distance_matrix[from_node.value()][to_node.value()] +
//					inst.nodes.at(from_node.value())->service_duration;
//				if (from_node.value() >= 2 * (data.requests.size() + data.vehicle_capacities.size()))
//					transit_time += data.max_fuel - current_battery;
//				return transit_time;
//		};
//		return RoutingModel::MakeStateDependentTransit(fuel_dependent_transit_time, 0,inst.Horizon);
//	};
//	
//	const int fuel_dependent_time_callback_index = routing.RegisterStateDependentTransitCallback(fuel_dependent_time_evaluator);
//	//Add Time Dimension
//	std::string time{ "Time" };
//	
//	routing.AddDimensionDependentDimensionWithVehicleCapacity(fuel_dependent_time_callback_index,
//		fuel_dimension,
//		inst.Horizon,
//		inst.Horizon,
//		false,
//		time);
//	RoutingDimension* time_dimension = routing.GetMutableDimension(time);
//
//	
//	
//
//	//With this dimension we calculate the violation of ride time constraints along the route
//	routing.AddConstantDimensionWithSlack(0,inst.Horizon,inst.Horizon,true,"RideTime");
//	RoutingDimension* ride_time_dimension = routing.GetMutableDimension("RideTime");
//
//	//Define the problem's requests
//	Solver* const solver = routing.solver();
//	for (const auto& request : data.requests) {
//		int64_t pickup_index = manager.NodeToIndex(request.at(0));
//		int64_t delivery_index = manager.NodeToIndex(request.at(1));
//		//Pairing Constraint
//		solver->AddConstraint(solver->MakeEquality(
//			routing.VehicleVar(pickup_index), routing.VehicleVar(delivery_index)));
//		//Precedence Constraint
//		solver->AddConstraint(
//			solver->MakeLessOrEqual(time_dimension->CumulVar(pickup_index),
//				time_dimension->CumulVar(delivery_index)));
//		routing.AddPickupAndDelivery(pickup_index, delivery_index);
//		//Time Window Soft Constraints
//		time_dimension->CumulVar(pickup_index)->SetMin(data.time_windows[request.at(0).value()].first);
//		time_dimension->SetCumulVarSoftUpperBound(pickup_index, data.time_windows[request.at(0).value()].second, 1);
//		time_dimension->CumulVar(delivery_index)->SetMin(data.time_windows[request.at(1).value()].first);
//		time_dimension->SetCumulVarSoftUpperBound(delivery_index, data.time_windows[request.at(1).value()].second, 1);
//		
//		//Ride Time Soft Constraints
//		int excessFlowTime = time_dimension->CumulVar(delivery_index) -
//			time_dimension->CumulVar(pickup_index) - data.maximum_ride_times.at(request.at(0).value());
//		solver->AddConstraint(solver->MakeGreaterOrEqual(ride_time_dimension->SlackVar(delivery_index), excessFlowTime));
//
//
//		//We are instructing the model to not regain fuel when visiting request nodes
//		fuel_dimension->SlackVar(pickup_index)->SetValue(0);
//		fuel_dimension->SlackVar(delivery_index)->SetValue(0);
//
//		routing.AddVariableMinimizedByFinalizer(fuel_dimension->CumulVar(pickup_index));
//		routing.AddVariableMinimizedByFinalizer(fuel_dimension->CumulVar(delivery_index));
//	}
//
//
//	//Set each vehicle's start time as hard constraint and end time as soft.
//	for (size_t i = 0; i < data.vehicle_capacities.size(); ++i)
//	{
//		EAV* vehicle = inst.vehicles.at(i);
//		int64_t start_index = routing.Start(i);
//		int64_t end_index = routing.End(i);
//
//		//The total ride time violation of the route is stored in the cumulative variable at the ending depot.
//		ride_time_dimension->SetCumulVarSoftUpperBound(end_index, 0, 1);
//
//		int start_node = manager.IndexToNode(start_index).value();
//		int end_node = manager.IndexToNode(end_index).value();
//		fuel_dimension->CumulVar(start_index)->SetValue(data.max_fuel);
//		time_dimension->CumulVar(start_index)->SetMin(data.time_windows.at(start_node).first);
//		time_dimension->SetCumulVarSoftUpperBound(end_index, data.time_windows.at(end_node).second , 1);
//		int desired_returned_battery = vehicle->battery_return_percentage * data.max_fuel;
//		fuel_dimension->SetCumulVarSoftLowerBound(end_index,desired_returned_battery,1);
//		fuel_dimension->SlackVar(start_index)->SetValue(0);
//
//		routing.AddVariableMinimizedByFinalizer(time_dimension->CumulVar(start_index));
//		routing.AddVariableMinimizedByFinalizer(time_dimension->CumulVar(end_index));
//		
//	}
//
//	//Replenish fuel when visiting a charging station
//	for (size_t x = 0; x < data.charging_stations.size(); x++)
//	{
//		int64_t index = manager.NodeToIndex(data.charging_stations.at(x));
//		CStation* s = inst.getChargingStation(inst.nodes.at(data.charging_stations.at(x).value()));
//		routing.AddDisjunction({index}, 0);
//
//	}
//	
//
//
//	// Setting first solution heuristic.
//	RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
//	searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
//	// Solve the problem.
//	const Assignment* solution = routing.SolveWithParameters(searchParameters);
//	
//
//	if (solution != nullptr) {
//		PrintSolution(data, manager, routing, *solution);
//	}
//	else {
//		LOG(INFO) << "No Solution Found";
//	}
//}

void VehicleExclusionImpact() {
	unsigned int initialRequests;
	for (int x = 0; x <= 4; x++) {
		ofstream timeFile("exclude" + to_string(x) + "time.txt");
		ofstream costFile("exclude" + to_string(x) + "cost.txt");
		timeFile << "Instance Time\n";
		costFile << "Instance Cost\n";
		printf("%s%i%s\n", "Excluding ", 100 * x / 4, "% of vehicles...");
		std::pair<int, int> vehicleRange{ 9,16 };
		initialRequests = 108;
		inst.controlParameter = x;
		for (int i = vehicleRange.first; i <= vehicleRange.second; i++) {
			inst.loadFromFile("Instances-MDHDARP/a" + to_string(i) + "-" + to_string(initialRequests) + "hetIUY.txt", 3);
			printf("%s%i%c%i\n", "Instance ", i, '-', initialRequests);
			double avgTime;
			double avgCost;
			for (int j = 0; j < 5; j++) {
				Run run = algorithms::IteratedGreedy(algorithms::details::Init1(), 130, INT_MAX);
				avgTime += run.elapsed_seconds;
				avgCost += run.best.AugmentedTchebycheff(0.3);
			}
			avgTime /= 5;
			avgCost /= 5;
			timeFile << i - 8 << " " << avgTime << "\n";
			costFile << i - 8 << " " << avgCost << "\n";
			initialRequests += 12;
			inst.~Instance();
		}
		timeFile.close();
		costFile.close();
	}
}

int main() {

	inst.loadFromFile("Instances-MDHDARP/a10-100hetIUY.txt", 3);
	inst.controlParameter = 4;
	Solution init = algorithms::details::Init1();
	Run run = algorithms::IteratedGreedy(init, 100, 240);
	printf("%s%f\n","CPU(s): ", run.elapsed_seconds);
	run.init.Display(2);
	run.best.Display(2);
	
	return EXIT_SUCCESS;
}
