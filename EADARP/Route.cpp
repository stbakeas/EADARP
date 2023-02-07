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
    for (CStation* s : inst.charging_stations) { 
        charging_times[s] = 0;
        assigned_cs[s] = false;
    }
    policy = ChargingPolicy::MINIMAL;
    adaptiveCharging = true;
}

double Route::getEarliestTime(int i) {
    if (i == 0)
        return vehicle->start_time;
    else
        return std::max(
            path[i]->earliest,
            getEarliestTime(i - 1) + path[i-1]->service_duration+ inst.getTravelTime(path[i-1], path[i])
        );
}

int Route::getLoad(int i) {
    return (i == 0) ? 0 : getLoad(i - 1) + path[i]->load;
}

bool Route::isEmpty() {
    return path.size() == 0;
}

bool Route::hasNoRequests() {
    if (path.size() <= 2) return true;
    else {
        for (size_t i = 0; i < path.size(); i++) if (path[i]->isOrigin()) return false;
        return true;
    }
    
}

bool Route::isFeasible() {
    return batteryFeasible && capacityFeasible;
}

void Route::BasicScheduling() {
    int n = (int)path.size();
    arrival_times.clear();
    arrival_times.resize(n);
    modified_start_of_service_times.clear();
    modified_start_of_service_times.resize(n);
    departure_times.clear();
    departure_times.resize(n);
    waiting_times.clear();
    waiting_times.resize(n);
    start_of_service_times.clear();
    start_of_service_times.resize(n);
    ride_times.clear();
    ride_times.resize(n);
    stay_times.clear();
    stay_times.resize(n);

    for (int i = 0; i < n; i++)
    {
        computeArrivalTime(i);
        computeStartOfServiceTime(i);
        computeModifiedStartOfServiceTime(i);
        computeWaitingTime(i);
        computeStayTime(i);
        computeDepartureTime(i);
        computeRideTime(i);
    }
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
        double pj = 0.0;

        if (path[j]->isDestination() && node_indices[inst.getRequest(path[j])->origin] < i)
            pj = ride_times[j];

        double latest_j = (j == path.size() - 1) ? vehicle->end_time : path[j]->latest;
        double time_slack = std::accumulate(waiting_times.begin() + i+1, waiting_times.begin() + j+1, 0.0) +
            std::max(0.0, std::min(latest_j - start_of_service_times[j], path[j]->maximum_travel_time - pj));

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
    }


    return min_time_slack;
}

void Route::LazyScheduling() {
    int n = (int)path.size();
    double forward_time_slack_at_0 = get_forward_time_slack(0);

    // STEP 4
    departure_times[0] = vehicle->start_time +  std::min(
        forward_time_slack_at_0, std::accumulate(waiting_times.begin(), waiting_times.end() - 1, 0.0)
    );

    start_of_service_times[0] = departure_times[0];

    // STEP 5
    for (int i = 1; i < path.size(); i++) {
        computeArrivalTime(i);
        computeStartOfServiceTime(i);
        computeModifiedStartOfServiceTime(i);
        computeWaitingTime(i);
        computeStayTime(i);
        computeDepartureTime(i);
    }

    // STEP 6
    for (int i = 1; i < path.size() - 1; i++) computeRideTime(i);

    // STEP 7
    for (int j = 1; j < path.size() - 1; j++) {
            // STEP 7 (a)
            double forward_time_slack =get_forward_time_slack(j);

            // STEP 7 (b)
            waiting_times[j] += std::min(
                forward_time_slack, std::accumulate(waiting_times.begin() + j + 1, waiting_times.end() - 1, 0.0)
            );

            start_of_service_times[j] = arrival_times[j] + waiting_times[j];
            computeDepartureTime(j);
            

            // STEP 7 (c):
            for (int i = j + 1; i < path.size(); i++) {
                computeArrivalTime(i);
                computeStartOfServiceTime(i);
                computeModifiedStartOfServiceTime(i);
                computeWaitingTime(i);
                computeStayTime(i);
                computeDepartureTime(i);
            }

            // STEP 7 (d):
            for (int i = j + 1; i < path.size() - 1; i++) computeRideTime(i);
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
    if (adaptiveCharging) LazyScheduling();
    storeNaturalSequences();
    computeTotalCost();
}

void Route::computeLoad(int i) {
    loads[i] = (i == 0) ? 0 : loads[i-1] + path[i]->load;
}

void Route::computeArrivalTime(int i)
{
    if (i == 0) {
        arrival_times[i] = vehicle->start_time;
    }
    else {
        arrival_times[i] = std::max(arrival_times[i-1], path[i-1]->earliest)
            + path[i-1]->service_duration
            + inst.getTravelTime(path[i-1], path[i]);
    }   
}

void Route::computeModifiedStartOfServiceTime(int i) {
    if (arrival_times[i] <= path[i]->latest) {
        modified_start_of_service_times[i] = start_of_service_times[i];
    }
    else {
        modified_start_of_service_times[i] = path[i]->latest;
    }
}

void Route::computeStartOfServiceTime(int i)
{
    start_of_service_times[i] = std::max(arrival_times[i], path[i]->earliest);
}

void Route::computeWaitingTime(int i)
{
    waiting_times[i] = start_of_service_times[i] - arrival_times[i];
}

void Route::computeDepartureTime(int i)
{
    departure_times[i] = start_of_service_times[i] + path[i]->service_duration;
    if (path[i]->isChargingStation()) departure_times[i] += stay_times[i];
}

void Route::computeRideTime(int i)
{
    if (!path[i]->isDestination()) ride_times[i] = 0;
    else {
        ride_times[i] = start_of_service_times[i] - departure_times[node_indices[inst.getRequest(path[i])->origin]];
    }
    
}

void Route::computeStayTime(int i) {
    if (path[i]->isChargingStation()) {
        CStation* s = inst.getChargingStation(path[i]);
        stay_times[i] = charging_times[s];
    }
    else stay_times[i] = waiting_times[i]+path[i]->service_duration;
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
        if (path[i]->isChargingStation()) {
            CStation* cs = inst.getChargingStation(path[i]);
            cost[static_cast<int>(Instance::Objective::System)] += charging_times[cs] * cs->recharge_cost;
        }
        cost[static_cast<int>(Instance::Objective::User)] += std::max(0.0, start_of_service_times[i] - path[i]->latest) +
            std::max(0.0, ride_times[i] - path[i]->maximum_travel_time);
    }

    for (Request* r : requests)  cost[static_cast<int>(Instance::Objective::System)] -= r->reward;

    if (loads.back() > vehicle->capacity) {
        capacityFeasible = false;
        return;
    }
    if (battery.back() < vehicle->battery*vehicle->battery_return_percentage) {
        batteryFeasible = false;
        return;
    }
    
    cost[static_cast<int>(Instance::Objective::Owner)] += std::max(0.0, arrival_times.back() - path.back()->latest);
}
//Inserting node after position i
double Route::getAddedDistance(Node* node, int i, Measure measure) const {
    double dist = inst.getTravelTime(path[i], node)
        + inst.getTravelTime(node, path[i+1])
        - inst.getTravelTime(path[i], path[i+1]);

    switch (measure) {
    case Measure::Time: {
        double arrival_time = departure_times[i] + inst.getTravelTime(path[i], node);
        double waiting_time = std::max(0.0, node->earliest - arrival_time);
        dist += waiting_time + node->service_duration;
        break;
    }
    case Measure::Battery: {
        dist *= inst.dischargeRate;
        break;
    }
    }

    return dist;
}
//Inserting origin after i and destination after j.
double Route::getAddedDistance(Request* request, int i, int j, Measure measure) const {
    if (i == j) {
        double dist = inst.getTravelTime(path[i], request->origin)
            + inst.getTravelTime(request->origin, request->destination)
            + inst.getTravelTime(request->destination, path[i+1])
            - inst.getTravelTime(path[i], path[i+1]);
        switch (measure) {
        case Measure::Time:
        {
            double arrival_time_origin = departure_times[i] + inst.getTravelTime(path[i], request->origin);
            double waiting_time_origin = std::max(0.0, request->origin->earliest - arrival_time_origin);
            double arrival_time_dest = arrival_time_origin + waiting_time_origin + request->origin->service_duration +
                inst.getTravelTime(request->origin, request->destination);
            double waiting_time_dest = std::max(0.0, request->destination->earliest - arrival_time_dest);
            dist += waiting_time_origin + waiting_time_dest + 2 * request->origin->service_duration;
            break;
        }
        case Measure::Battery:
            dist *= inst.dischargeRate;
            break;
        }
        return dist;
    }
    else return getAddedDistance(request->origin, i, measure) + getAddedDistance(request->destination, j, measure);
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
                return battery[closest_right] - getAddedDistance(request, i, j, Measure::Battery) >= limit; 
            }
            else {
                CStation* s = inst.getChargingStation(path[closest_left]);
                return s->getChargedAmount(charging_times[s])
                    + getAddedDistance(request, i, j, Measure::Battery) <= vehicle->battery;
            }
        }
        else {
            double battery_after_insertion;
            double added_battery_consumption = getAddedDistance(request, i, j, Measure::Battery);
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
