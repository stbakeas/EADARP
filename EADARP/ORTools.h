#pragma once
#include <vector>
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace ORTools {
	
	using namespace operations_research;

	struct DataModel {
		std::vector < std::vector<int>> distance_matrix;
		std::vector<std::pair<int, int>> time_windows;
		std::vector <std::vector<RoutingIndexManager::NodeIndex>> requests;
		std::vector<RoutingIndexManager::NodeIndex> start_depots;
		std::vector<RoutingIndexManager::NodeIndex> end_depots;
		std::vector<RoutingIndexManager::NodeIndex> charging_stations;
		std::vector<int64_t> vehicle_capacities;
		std::vector<int> maximum_ride_times;
		int max_fuel = 180;
	};

	void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
		const RoutingModel& routing, const Assignment& solution);
}

