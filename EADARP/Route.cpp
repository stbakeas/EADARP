#include "Route.h"
#include <iostream>
#include "Instance.h"
#include "CStation.h"
#include <numeric>
#include <limits>
#include <algorithm>
#include "RandLib.h"
#include "Position.h"

double dbl_round(long double number, int precision) noexcept {
    static int pow10[5] = {
    1, 10, 100,1000,10000
    };
    return std::round(number * pow10[precision]) / pow10[precision];
}

Route::Route(EAV* vehicle) :
    batteryFeasible(true),
    capacityFeasible(true),
    timeFeasible(true),
    policy(ChargingPolicy::MINIMAL),
    vehicle(vehicle)
{}

void Route::BasicScheduling() {
    int n = path.size();
    start_of_service_times.clear();
    start_of_service_times.resize(n);
    FTS.clear();
    FTS.resize(n);
    waiting_times.clear();
    waiting_times.resize(n);


    start_of_service_times[0] = vehicle->start_time;
    waiting_times[0] = 0.0;
    for (int i = 1; i < n; i++) {
        computeStartOfServiceTime(i);
        waiting_times[i] = getWaitingTime(i);
    }

    FTS[n - 1] = std::max(0.0, vehicle->end_time - start_of_service_times[n - 1]);
    for (int i = n-2; i!=-1; i--) {
        FTS[i] = waiting_times[i] + std::min(FTS[i + 1], std::max(0.0, path[i]->latest - start_of_service_times[i]));
    }
    
}

void Route::storeNaturalSequences()
{
    natural_sequences.clear();
    for (size_t i = 0; i < path.size(); i++) {
        std::pair<int, int> bounds;
        if (loads[i]) {
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
    double cumulWaitingTime = 0.0;
    for (int j = i; j < path.size(); j++) {
        double ride_time = 0.0, max_travel_time=path[j]->maximum_travel_time;
        double latest_j = (j == path.size() - 1) ? vehicle->end_time : path[j]->latest;
        
        if (path[j]->isDestination()) {
            int origin_index = node_indices[inst.getRequest(path[j])->origin];
            if (origin_index < i) {
                ride_time = getRideTime(origin_index);
                max_travel_time = path[origin_index]->maximum_travel_time;
            }
        }

        double time_slack = cumulWaitingTime +
            std::max(0.0, std::min(latest_j - start_of_service_times[j], max_travel_time - ride_time));

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
        
        if (j!=path.size()-1) cumulWaitingTime += waiting_times[j + 1];
    }

    return min_time_slack;
}

double Route::get_modified_time_slack(int i) {
    double min_time_slack = DBL_MAX;
    double cumul_waiting_time = 0.0;
    for (int j = i; j < path.size(); j++) {
        double time_slack;
        if (path[j]->isDestination() && node_indices[inst.getRequest(path[j])->origin] < i)
            time_slack = cumul_waiting_time;
        else
            time_slack = cumul_waiting_time + std::max(0.0, (j == path.size() - 1) ? vehicle->end_time : path[j]->latest -start_of_service_times[j]);

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
        
    }

    return min_time_slack;
}

double Route::getWaitingTime(int index) {
   
     return !path[index - 1]->isChargingStation()?start_of_service_times[index] - (start_of_service_times[index - 1] + path[index - 1]->service_duration + inst.getTravelTime(path[index-1], path[index])) :
         start_of_service_times[index] - (start_of_service_times[index - 1] + inst.getChargingStation(path[index - 1])->getRequiredTime(battery[index - 1], departureBatteryLevel[inst.getChargingStation(path[index - 1])]) + inst.getTravelTime(path[index-1], path[index]));
}

double Route::getRideTime(int index) {
    return path[index]->isOrigin() ?
        start_of_service_times[node_indices[inst.getRequest(path[index])->destination]]-
        (start_of_service_times[index]+path[index]->service_duration): 0.0;
}

void Route::LazyScheduling() {
    size_t n = path.size();

    start_of_service_times[0] = vehicle->start_time+std::min(get_forward_time_slack(0),
        std::accumulate(waiting_times.begin()+1,waiting_times.end()-1,0.0));

    for (int i = 1; i < n; i++) {
        computeStartOfServiceTime(i);
        waiting_times[i] = getWaitingTime(i);
    }

    // STEP 7
    for (int j = 1; j < n - 1; j++) {
        if (path[j]->isOrigin()) {

           start_of_service_times[j]+= std::min(get_forward_time_slack(j),
               std::accumulate(waiting_times.begin() + j + 1, waiting_times.end() - 1, 0.0));

            // STEP 7 (c):
           for (int i = j + 1; i < n; i++) {
               computeStartOfServiceTime(i);
               waiting_times[i] = getWaitingTime(i);
           }
        }
    }

}

void Route::RideTimeOrientedScheduling()
{
    
}

void Route::updateMetrics(bool debug) {
    batteryFeasible = capacityFeasible = timeFeasible = true;
    size_t n = path.size();
    loads.clear();
    loads.resize(n);
    battery.clear();
    battery.resize(n);
    battery[0] = vehicle->initial_battery;
    loads[0] = 0;
    for (int i = 1; i < n; i++) {
        computeLoad(i);
        computeBatteryLevel(i);
        computeChargingTime(i);
    }
    BasicScheduling();
    LazyScheduling();
    storeNaturalSequences();
    computeTotalCost(debug);
}

void Route::computeLoad(int i) {
    loads[i] = loads[i-1] + path[i]->load;
}

void Route::computeStartOfServiceTime(int i)
{
    start_of_service_times[i]=!path[i-1]->isChargingStation()? std::max(path[i]->earliest,
            start_of_service_times[i - 1] + path[i-1]->service_duration + inst.getTravelTime(path[i - 1], path[i])):
        std::max(path[i]->earliest,
            start_of_service_times[i - 1] + inst.getChargingStation(path[i-1])->getRequiredTime(battery[i-1], departureBatteryLevel[inst.getChargingStation(path[i-1])]) + inst.getTravelTime(path[i-1], path[i]));
     
}

bool Route::deleteRedundantChargingStations()
{ 
    bool routeChanged = false;
    for (const auto& [station,amount]:departureBatteryLevel){
        Node* cs_node = inst.nodes[station->id - 1];
        double charging_time = station->getRequiredTime(battery[node_indices[cs_node]], amount);
        if (charging_time==0.0) {
          removeNode(node_indices[cs_node]);
          routeChanged = true;
        }
    }
    return routeChanged;
}

void Route::computeBatteryLevel(int i) {
    battery[i] = battery[i-1] - inst.getFuelConsumption(path[i-1], path[i]);
    if (path[i-1]->isChargingStation()) {
        CStation* s = inst.getChargingStation(path[i-1]);
        battery[i] += s->getRequiredTime(battery[i-1],departureBatteryLevel[s]) * s->recharge_rate;
    }
    
}

void Route::computeTotalCost(bool debug) {
    int n = path.size();
    excess_ride_time = 0.0;

    for (int i = 0; i < n - 1; i++) {
        if (loads[i] > vehicle->capacity) {
            capacityFeasible = false;
            if (debug) {
                std::cout << "Capacity Infeasible" << std::endl;
            }
            return;
        }

        if (dbl_round(battery[i], 3) < 0.0) {
            batteryFeasible = false;
            if (debug) {
                std::cout << "Battery Infeasible: " << dbl_round(battery[i], 2) << std::endl;
            }
            return;
        }
        if (dbl_round(start_of_service_times[i] - path[i]->latest, 3) > 0.0 || dbl_round(getRideTime(i) - path[i]->maximum_travel_time, 3) > 0.0) {
            timeFeasible = false;
            if (debug) {
                std::cout << "Time Window:" << dbl_round(start_of_service_times[i] - path[i]->latest, 3) << "\n";
                std::cout << "Ride Time:" << getRideTime(i) << "\n";
            }
            return;
        }

        if (path[i]->isOrigin()) excess_ride_time += getRideTime(i) - inst.getTravelTime(path[i], inst.getRequest(path[i])->destination);

    }

    if (loads[n - 1] > vehicle->capacity) {
        capacityFeasible = false;
        if (debug) {
            std::cout << "Capacity Infeasible" << std::endl;
        }
        return;
    }
    if (dbl_round(battery[n - 1], 3) < dbl_round(vehicle->minBatteryUponReturn, 3)) {
        batteryFeasible = false;
        if (debug) {
            std::cout << "Return Battery Infeasible: " << dbl_round(battery[n - 1] - vehicle->minBatteryUponReturn, 3) << "\n";
        }
        return;
    }
    if (dbl_round(vehicle->end_time - start_of_service_times[n - 1], 3) < 0.0) {
        timeFeasible = false;
        if (debug) {
            std::cout << "Time Window:" << dbl_round(start_of_service_times[n - 1] - vehicle->end_time, 3) << "\n";
        }
    }
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
    return start_of_service_times.back() - start_of_service_times[0];
}

//This function inserts a node in a route before the specified index
void Route::insertNode(Node* node, int index)
{
    if (index < 1)
        perror("Error! Can not insert a node before the starting depot");
    else if (index > path.size()) perror("Error! Can not insert a node after the ending depot");
    else { 
        std::for_each(path.begin() + index, path.end(), [&](Node* node) {node_indices[node]++; });
        travel_distance += getAddedDistance(node,index-1);
        path.insert(path.begin() + index, node);
        node_indices[node] = index;
        if (node->isChargingStation()) departureBatteryLevel[inst.getChargingStation(node)] = 0.0;
    }
}

void Route::insertRequest(Request* r, int i, int j) {
    insertNode(r->origin, i);
    insertNode(r->destination, j + 1);
    requests.push_back(r);
}

void Route::computeChargingTime(int nodePosition) {
    if (!path[nodePosition]->isChargingStation()) return;
    CStation* s = inst.getChargingStation(path[nodePosition]);
    double desired_amount = 0.0;
    //Calculate the fuel to be added in order to reach a charging station or the ending depot.
    Node* current_node = path[nodePosition];
    for (size_t x = nodePosition + 1; x < path.size(); x++) {
        desired_amount += inst.getFuelConsumption(current_node, path[x]);
        if (path[x]->isChargingStation()) break;
        current_node = path[x];
    }
    departureBatteryLevel[s]=std::min(desired_amount+current_node->isEndingDepot()*vehicle->minBatteryUponReturn, vehicle->total_battery);
}

std::pair<size_t, size_t> Route::getRequestPosition(const Request* r) {
    std::pair<size_t,size_t> positions;
    if (node_indices.contains(r->origin) && node_indices.contains(r->destination)) {
        positions.first = node_indices[r->origin];
        positions.second = node_indices[r->destination];
    }
    else perror("Error! This request is not served during this route");
    return std::move(positions);
}

CStation* Route::findBestChargingStationAfter(int i,std::unordered_map<CStation*,unsigned int>& stationVisits) {
    //Find all the reachable stations from the zero-load node
    std::vector<CStation*> reachable_stations;
    reachable_stations.reserve(inst.charging_stations.size());
    std::copy_if(inst.charging_stations.begin(),
        inst.charging_stations.end(),
        std::back_inserter(reachable_stations),
        [&stationVisits,i,this](CStation* station) {return stationVisits[station] < inst.maxVisitsPerStation &&
        !departureBatteryLevel.contains(station) &&
        battery[i] - inst.getFuelConsumption(path[i], inst.nodes[station->id - 1]) >= 0.0; }
    );
   
    //Find the charging station with the minimum distance that can be reached from the node.
    return reachable_stations.empty()?nullptr:*std::min_element(reachable_stations.begin(), reachable_stations.end(), 
        [&](const CStation* cs1, const CStation* cs2) 
        {
            return  inst.dischargeRate * getAddedDistance(inst.nodes[cs1->id - 1], i) <
                inst.dischargeRate * getAddedDistance(inst.nodes[cs2->id - 1], i);
        }
    );
    
}

void Route::removeNode(int index){
    if (index > 0 && index < path.size()) {
       node_indices.erase(path[index]);
       travel_distance=travel_distance-inst.getTravelTime(path[index - 1], path[index])-
            inst.getTravelTime(path[index], path[index + 1]) +
            inst.getTravelTime(path[index - 1], path[index + 1]);
        departureBatteryLevel.erase(inst.getChargingStation(path[index]));
        path.erase(path.begin() + index);
        std::for_each(path.begin() + index, path.end(),
            [&](Node* node) {node_indices[node]--; }
        );
       
    }
}

void Route::removeRequest(Request* request)
{
    std::pair < size_t, size_t > indices= getRequestPosition(request);
    if (path[indices.first]->id != request->origin->id || path[indices.second]->id != request->destination->id)
        std::cout << "Error in removal." << "\n";
    removeNode(indices.first);
    removeNode(indices.second-1);
    requests.erase(std::remove(requests.begin(), requests.end(), request),requests.end());
}

bool Route::isInsertionCapacityFeasible(Request* request, int i, int j) const {
    return std::none_of(path.begin() + i, path.begin() + j + 1, [request, this](Node* node) {
        return loads[node_indices.at(node)] + request->origin->load > vehicle->capacity && node->isChargingStation();
    });
}

/*
  Checks that the battery level at the assigned charging stations and the depot is above 0.
  Worst case running time: O(|S|), where S the set of available charging stations. 
*/
bool Route::isInsertionBatteryFeasible(Request* request, int i, int j,bool increaseChargingTime) {
  
     double added_battery_consumption = getAddedDistance(request, i, j) * inst.dischargeRate;
     if (increaseChargingTime) {
         //Spot the important nodes (depots,charging stations) the request lies between.
         int closest_left = 0,closest_right = path.size() - 1;
         for (const auto& [station,amount]:departureBatteryLevel) {
             int index = node_indices[inst.nodes[station->id - 1]];
             if (unsigned(index-closest_left)<=i-closest_left) closest_left = index;
             if (unsigned(index-j)< closest_right-j) closest_right = index;
         }
         
         if (!closest_left || policy == ChargingPolicy::FULL) return battery[closest_right] - added_battery_consumption >= (closest_right == path.size() - 1)*vehicle->minBatteryUponReturn;
         
         return std::max(0.0,departureBatteryLevel[inst.getChargingStation(path[closest_left])]-battery[closest_left])
             + added_battery_consumption <= vehicle->total_battery;
         
     }

     
     return std::none_of(departureBatteryLevel.begin(), departureBatteryLevel.end(), [&](std::pair<CStation*,double> pair)
         {
             int index = node_indices[inst.nodes[pair.first->id - 1]];
             return index > j && battery[index] - added_battery_consumption < 0.0;
         }
     ) && battery.back() - added_battery_consumption >= vehicle->minBatteryUponReturn;
}

bool Route::isInsertionTimeFeasible(Request* request, int i, int j)
{
    //Check if a charging station is located before the request.
    double new_bos_i = start_of_service_times[i];
    int closest = 0;
    for (const auto& [station,amount]:departureBatteryLevel) {
         int csIndex = node_indices[inst.nodes[station->id - 1]];
         if (unsigned(csIndex-closest)<=i-closest) closest = csIndex;
    }

    if (closest) {
        CStation* cs = inst.getChargingStation(path[closest]);
        double added_charging_time = cs->getRequiredTime(battery[closest], departureBatteryLevel[cs] + getAddedDistance(request, i, j) * inst.dischargeRate);
        if (added_charging_time > FTS[closest+1]) return false;
        new_bos_i += std::max(0.0, added_charging_time - std::accumulate(waiting_times.begin() + closest + 1, waiting_times.begin() + i+1, 0.0));
    }

    double bos_origin = std::max(request->origin->earliest, new_bos_i + path[i]->service_duration + inst.getTravelTime(path[i], request->origin));
    if (bos_origin > request->origin->latest) return false;
    double added_time,bos_destination;

    if (i == j) {
       bos_destination = std::max(request->destination->earliest,bos_origin+request->origin->service_duration+
               inst.getTravelTime(request->origin, request->destination));
    }
    else {
        added_time = std::max(0.0, bos_origin + request->origin->service_duration + inst.getTravelTime(request->origin, path[i + 1]) 
            - start_of_service_times[i+1]);
        if (added_time > FTS[i + 1]) return false;

        double new_bos_j = start_of_service_times[j] +
            std::max(0.0, added_time - std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + j + 1, 0.0));
        bos_destination = std::max(request->destination->earliest, new_bos_j + path[j]->service_duration + 
               inst.getTravelTime(path[j],request->destination));
    }
    if (bos_destination > request->destination->latest) return false;
    added_time = std::max(0.0,
        bos_destination + request->destination->service_duration + inst.getTravelTime(request->destination, path[j + 1]) - start_of_service_times[j + 1]);
    return added_time <= FTS[j + 1];
}


