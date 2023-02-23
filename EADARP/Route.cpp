#include "Route.h"
#include <iostream>
#include "Instance.h"
#include "CStation.h"
#include <numeric>
#include <limits>
#include <algorithm>
#include "RandLib.h"
#include "Position.h"

Route::Route(EAV* vehicle) {
	this->vehicle = vehicle;
    batteryFeasible = true;
    capacityFeasible = true;
    timeFeasible = true;
    for (CStation* s : inst.charging_stations) { 
        charging_times[s] = 0;
        assigned_cs[s] = false;
    }
    policy = ChargingPolicy::MINIMAL;
    adaptiveCharging = true;
}

int Route::getLoad(int i) {
    return (i == 0) ? 0 : getLoad(i - 1) + path[i]->load;
}

bool Route::isEmpty() {
    return path.size() == 0;
}

bool Route::hasNoRequests() {
    return !requests.size();
}

bool Route::isFeasible() {
    return batteryFeasible && capacityFeasible && timeFeasible;
}

void Route::BasicScheduling() {
    int n = (int)path.size();
    start_of_service_times.clear();
    start_of_service_times.resize(n);

    start_of_service_times[0] = vehicle->start_time;
    for (int i = 1; i < n; i++) computeStartOfServiceTime(i);
}

void Route::storeNaturalSequences()
{
    natural_sequences.clear();
    for (size_t i = 0; i < path.size(); i++) {
        if (loads[i]) {
            std::pair<int, int> bounds;
            bounds.first = i;
            while (loads[i]) i++;
            bounds.second = i;
            natural_sequences.push_back(bounds);
        }
    }
}

double Route::get_forward_time_slack(int i)
{
    double min_time_slack = DBL_MAX;

    for (int j = i; j < path.size(); j++) {
        double latest_j = (j == path.size() - 1) ? vehicle->end_time : path[j]->latest;
        double cumul_waiting_time = 0.0;
        for (int x = i + 1; x <= j; x++) cumul_waiting_time += getWaitingTime(x);
        
        double time_slack = cumul_waiting_time +
            std::max(0.0, std::min(latest_j - start_of_service_times[j], path[j]->maximum_travel_time - getRideTime(j)));

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
    }


    return min_time_slack;
}

double Route::getWaitingTime(int index) {
    return (!index) ? 0 : start_of_service_times[index] -
        (start_of_service_times[index - 1] + path[index - 1]->service_duration + inst.getTravelTime(path[index - 1], path[index]));
}

double Route::getRideTime(int index) {
    return path[index]->isDestination() ? start_of_service_times[index]-(start_of_service_times[node_indices[inst.getRequest(path[index])->origin]]+
        inst.getRequest(path[index])->origin->service_duration): 0.0;
}

void Route::LazyScheduling() {
    int n = (int)path.size();
    double forward_time_slack_at_0 = get_forward_time_slack(0);

    double cumul_waiting_time = 0.0;
    for (int i = 1; i < n - 1; i++) cumul_waiting_time += getWaitingTime(i);

    // STEP 4
    start_of_service_times[0] = vehicle->start_time+std::min(forward_time_slack_at_0, cumul_waiting_time);

    // STEP 5
    for (int i = 1; i < path.size(); i++) computeStartOfServiceTime(i);

    // STEP 7
    for (int j = 1; j < path.size() - 1; j++) {
       // STEP 7 (a)
       double forward_time_slack =get_forward_time_slack(j);
       
       
       
       // STEP 7 (b)
       double w_j = getWaitingTime(j);
       cumul_waiting_time = 0.0;
       for (int x = j + 1; x < path.size(); x++) cumul_waiting_time += getWaitingTime(x);
       
       w_j += std::min(forward_time_slack, cumul_waiting_time);
       
       start_of_service_times[j] = start_of_service_times[j - 1] + path[j - 1]->service_duration
           + inst.getTravelTime(path[j - 1], path[j]) + w_j;
       
       // STEP 7 (c):
       for (int i = j + 1; i < path.size(); i++) computeStartOfServiceTime(i);
    }

}

void Route::updateMetrics() {
    batteryFeasible = true;
    capacityFeasible = true;
    int n = (int)path.size();
    loads.clear();
    loads.resize(n);
    battery.clear();
    battery.resize(n);
    requests.clear();
    if (adaptiveCharging) for (CStation* cs : inst.charging_stations) { assigned_cs[cs] = false; charging_times[cs] = 0; }
    BasicScheduling();
    for (int i = 0; i < n; i++) {
        node_indices[path[i]] = i;
        computeLoad(i);
        computeBatteryLevel(i);
        if (adaptiveCharging) computeChargingTime(i);
        if (path[i]->isOrigin()) requests.push_back(inst.getRequest(path[i]));
    }
    if (adaptiveCharging) BasicScheduling();
    storeNaturalSequences();
    computeTotalCost();
}

void Route::computeLoad(int i) {
    loads[i] = (i == 0) ? 0 : loads[i-1] + path[i]->load;
}

void Route::computeStartOfServiceTime(int i)
{
    double stay_time = path[i - 1]->service_duration;
    if (path[i - 1]->isChargingStation()) stay_time += charging_times[inst.getChargingStation(path[i - 1])];
    start_of_service_times[i] = std::max(path[i]->earliest,
        start_of_service_times[i - 1] + stay_time + inst.getTravelTime(path[i - 1], path[i]));
}

void Route::deleteRedundantChargingStations()
{
    for (auto pair:charging_times){
        if (!pair.second) {
            removeNode(node_indices[inst.nodes[pair.first->id - 1]]);
            updateMetrics();
        }
    }
}

void Route::computeBatteryLevel(int i) {
    if (i == 0) battery[i] = vehicle->battery;
    else {
        battery[i] = battery[i-1] - inst.getFuelConsumption(path[i-1], path[i]);
        if (path[i-1]->isChargingStation()) {
            CStation* s = inst.getChargingStation(path[i-1]);
            battery[i] += charging_times[s] * s->recharge_rate;
        }
    }
}

void Route::computeTotalCost() {
    int n = path.size();
    for (auto objective : inst.objectives) cost[static_cast<int>(objective)] = 0.0;

    for (int i = 1; i < n - 1; i++) {
        if (loads[i] > vehicle->capacity) {
            capacityFeasible = false;
            return;
        }
        if (battery[i] < 0) {
            batteryFeasible = false;
            return;
        }
        cost[static_cast<int>(Instance::Objective::User)] += std::max(0.0, start_of_service_times[i] - path[i]->latest) +
            std::max(0.0, getRideTime(i) - path[i]->maximum_travel_time);
    }

    cost[static_cast<int>(Instance::Objective::System)] -= requests.size();

    if (loads.back() > vehicle->capacity) {
        capacityFeasible = false;
        return;
    }
    if (battery.back() < vehicle->battery*vehicle->battery_return_percentage) {
        batteryFeasible = false;
        return;
    }
    if (start_of_service_times.back() > vehicle->end_time) {
        timeFeasible = false;
        return;
    }
    
    if (!hasNoRequests()) cost[static_cast<int>(Instance::Objective::Owner)] = vehicle->acquisition_cost;
}
//Inserting node after position i
double Route::getAddedDistance(Node* node, int i) const {
    return inst.getTravelTime(path[i], node)
        + inst.getTravelTime(node, path[i+1])
        - inst.getTravelTime(path[i], path[i+1]);
}

//Inserting origin after i and destination after j.
double Route::getAddedDistance(Request* request, int i, int j) const {
    if (i == j) return inst.getTravelTime(path[i], request->origin)
            + inst.getTravelTime(request->origin, request->destination)
            + inst.getTravelTime(request->destination, path[i+1])
            - inst.getTravelTime(path[i], path[i+1]);

    else return getAddedDistance(request->origin, i) + getAddedDistance(request->destination, j);
}

double Route::getDuration()
{
    return start_of_service_times.back() - start_of_service_times.front();
}

//This function inserts a node in a route before the specified index
void Route::insertNode(Node* node, int index)
{
    if (isEmpty()) path.push_back(node);
    else {
        if (index < 1)
            perror("Error! Can not insert a node before the starting depot");
        else if (index > path.size()) perror("Error! Can not insert a node after the ending depot");
        else { 
            node_indices[node] = index;
            path.insert(path.begin() + index, node);
            if (!adaptiveCharging && node->isChargingStation()) computeChargingTime(index);
        }
    }
   
}

void Route::insertRequest(Request* r, int i, int j) {
    insertNode(r->origin, i);
    insertNode(r->destination, j + 1);
}

void Route::computeChargingTime(int nodePosition) {
    if (path[nodePosition]->isChargingStation()) {
        double arrival_battery = battery[nodePosition];
        if(!adaptiveCharging) arrival_battery-=inst.getFuelConsumption(path[nodePosition - 1], path[nodePosition]);
        CStation* s = inst.getChargingStation(path[nodePosition]);
        if (assigned_cs[s]) batteryFeasible = false;
        else assigned_cs[s] = true;
        double desired_amount = 0.0;
        double extra_amount = 0.0;
        if (policy == ChargingPolicy::FULL)  desired_amount = vehicle->battery;
        else {
            //Calculate the fuel to be added in order to reach a charging station or the ending depot.
            double fuel_needed = 0.0;
            Node* current_node = path[nodePosition];
            for (size_t x = nodePosition + 1; x < path.size(); x++) {
                fuel_needed += inst.getFuelConsumption(current_node, path[x]);
                if (path[x]->isChargingStation()) break;
                else current_node = path[x];
            }
            if (policy == ChargingPolicy::CONSERVATIVE) {
                /* If maximum stay time has negative value,
                * it means we can not add extra charging time without violating time constraints at
                * subsequent nodes. */
                double maximum_stay_time;
                if (adaptiveCharging) {
                    maximum_stay_time = get_forward_time_slack(nodePosition + 1)
                        - s->getRequiredTime(battery[nodePosition], fuel_needed)
                        - inst.getTravelTime(path[nodePosition], path[nodePosition + 1]);
                }
                else {
                    maximum_stay_time = get_forward_time_slack(nodePosition) - (inst.getTravelTime(path[nodePosition - 1], path[nodePosition])
                        + inst.getTravelTime(path[nodePosition], path[nodePosition + 1])
                        - inst.getTravelTime(path[nodePosition - 1], path[nodePosition + 1]))
                        - s->getRequiredTime(arrival_battery, fuel_needed);
                }
                extra_amount = round(s->getChargedAmount(maximum_stay_time));
            }

            desired_amount = fuel_needed + extra_amount;
            if (current_node->isEndingDepot()) {
                double returned_battery = vehicle->battery_return_percentage * vehicle->battery;
                if (desired_amount - fuel_needed < returned_battery)
                    desired_amount += returned_battery;
            }
            desired_amount = std::min(desired_amount, vehicle->battery);
            
        }
        charging_times[s] = s->getRequiredTime(arrival_battery, desired_amount);
    }
}

std::pair<size_t, size_t> Route::getRequestPosition(const Request* r) {
    std::pair<size_t,size_t> positions;
    positions.first = node_indices[r->origin];
    positions.second = node_indices[r->destination];
    return positions;
}

CStation* Route::findBestChargingStationAfter(int i) {
    //Find all the reachable stations from the zero-load node
    Node* zero_load_node = path[i];
    std::vector<CStation*> reachable_stations;
    reachable_stations.reserve(inst.charging_stations.size());
    for (CStation* s : inst.charging_stations) {
        if (!assigned_cs[s]) {
            if (battery[i] -
                inst.getFuelConsumption(zero_load_node, inst.nodes[s->id - 1])
                >= 0) reachable_stations.push_back(s);
        }
    }
    if (!reachable_stations.empty())
    {
        //Find the charging station with the minimum distance that can be reached from the node.
        CStation* min_s = reachable_stations[0];
        double min_score = inst.dischargeRate*getAddedDistance(inst.nodes[min_s->id - 1], i);

        for (size_t x = 1; x < reachable_stations.size(); x++)
        {
            CStation* current_station = reachable_stations[x];
            double current_score = inst.dischargeRate*getAddedDistance(inst.nodes[current_station->id - 1], i);
            if (min_score > current_score) {
                min_s = current_station;
                min_score = current_score;
            }
        }
        return min_s;
    }
    return nullptr;
    
}

Request* Route::selectRandomRequest(RandLib randlib) {
    if (hasNoRequests()) return nullptr;
    else {
        return requests[randlib.randint(0, requests.size() - 1)];
    }
}

void Route::removeNode(int index)
{
    if (index > 0 && index < path.size() - 1) {
        Node* node = path[index];
        node_indices.erase(node);
        path.erase(path.begin() + index);
        if (!adaptiveCharging && node->isChargingStation()) {
            CStation* s = inst.getChargingStation(node);
            assigned_cs[s] = false;
            charging_times[s] = 0;
        }
    }
}

void Route::removeRequest(Request* request)
{
    std::pair < size_t, size_t > indices= getRequestPosition(request);
    removeNode(indices.first);
    removeNode(indices.second-1);
}

bool Route::isInsertionCapacityFeasible(Request* request, int i, int j) const {
    if (j < i || i < 0 || j < 0 || i + 1 >= path.size() || j + 1 >= path.size()) return false;
    else { 
        for (int x = i; x <= j; x++) {
            if (loads[x] + request->origin->load > vehicle->capacity) return false;
            if (loads[x] && path[x]->isChargingStation()) return false;
        }
        return true;
    }
}

/*
  Checks that the battery level at the assigned charging stations and the depot is above 0.
  Worst case running time: O(|S|), where S the set of available charging stations. 
*/
bool Route::isInsertionBatteryFeasible(Request* request, int i, int j,bool increaseChargingTime) {
    if (j < i || i < 0 || j < 0 || i + 1 >= path.size() || j + 1 >= path.size()) return false;
    else {
        if (increaseChargingTime) {
            //Spot the important nodes (depots,charging stations) the request lies between.
            int closest_left = 0;
            int closest_right = path.size() - 1;
            for (auto pair : assigned_cs) {
                if (pair.second) {
                    int index = node_indices[inst.nodes[pair.first->id - 1]];
                    if (index <= i && index > closest_left) closest_left = index;
                    if (index > j && index < closest_right) closest_right = index;
                }
            }
            double limit;
            if (!closest_left || policy == ChargingPolicy::FULL) { 
                limit = (closest_right == path.size() - 1) ?vehicle->battery*vehicle->battery_return_percentage:0.0;
                return battery[closest_right] - getAddedDistance(request, i, j)*inst.dischargeRate >= limit; 
            }
            else {
                CStation* s = inst.getChargingStation(path[closest_left]);
                return s->getChargedAmount(charging_times[s])
                    + getAddedDistance(request, i, j)*inst.dischargeRate <= vehicle->battery;
            }
        }
        else {
            double battery_after_insertion;
            double added_battery_consumption = getAddedDistance(request, i, j)*inst.dischargeRate;
            for (auto pair : assigned_cs) {
                if (pair.second) {
                    int index = node_indices[inst.nodes[pair.first->id - 1]];
                    if (index > j) {
                        battery_after_insertion = battery[index] - added_battery_consumption;
                        if (battery_after_insertion < 0.0) return false;
                    }
                }
            }
            battery_after_insertion = battery.back() - added_battery_consumption;
            return battery_after_insertion >= vehicle->battery_return_percentage*vehicle->battery;
        }
    }
}

bool Route::isInsertionTimeFeasible(Request* request, int i, int j)
{
    double cumul_waiting_time = 0.0;
    //Check if a charging station is located before the request.
    double new_bos_i = start_of_service_times[i];
    int closest = 0;
    for (const auto& [station, assigned] : assigned_cs) {
        if (assigned) {
            int csIndex = node_indices[inst.nodes.at(station->id - 1)];
            if (csIndex > closest && csIndex <=i) closest = csIndex;
        }
    }

    if (closest) {
        CStation* cs = inst.getChargingStation(path[closest]);
        double desiredAmount = std::min(vehicle->battery, cs->getChargedAmount(charging_times[cs]) + getAddedDistance(request, i, j)*inst.dischargeRate);
        double added_charging_time = cs->getRequiredTime(cs->getChargedAmount(charging_times[cs]),desiredAmount);
        cumul_waiting_time = 0.0;
        for (int x = closest + 1; x <= i; x++) cumul_waiting_time+=getWaitingTime(x);
        new_bos_i += std::max(0.0, added_charging_time - cumul_waiting_time);
    }

    double dep_origin = std::max(request->origin->earliest, new_bos_i + path[i]->service_duration + inst.getTravelTime(path[i], request->origin))
        + request->origin->service_duration;
    double added_time,dep_destination;

    if (i == j) {
        dep_destination = request->destination->service_duration +
            std::max(request->destination->earliest,dep_origin+
                inst.getTravelTime(request->origin, request->destination));

    }
    else {
        added_time = std::max(0.0, dep_origin + inst.getTravelTime(request->origin, path[i + 1]) 
            - start_of_service_times[i+1]);

        cumul_waiting_time = 0.0;
        for (int x = i + 1; x <= j; x++) cumul_waiting_time += getWaitingTime(x);

        double new_bos_j = start_of_service_times[j] + std::max(0.0, added_time - cumul_waiting_time);
        dep_destination = request->destination->service_duration +
            std::max(request->destination->earliest, new_bos_j + path[j]->service_duration + 
                inst.getTravelTime(path[j],request->destination));
    }
    added_time = std::max(0.0,
        dep_destination + inst.getTravelTime(request->destination, path[j + 1]) - start_of_service_times[j + 1]);

    cumul_waiting_time = 0.0;
    for (int x = j+1; x < path.size(); x++) cumul_waiting_time += getWaitingTime(x);   
    return start_of_service_times.back() + std::max(0.0, added_time - cumul_waiting_time) <= vehicle->end_time;
}


