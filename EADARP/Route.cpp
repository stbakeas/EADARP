#include "Route.h"
#include <iostream>
#include "Instance.h"
#include "CStation.h"
#include <numeric>
#include <limits>
#include <algorithm>
#include "RandLib.h"
#include "Position.h"

double dbl_round(long double number, int precision) {
    int decimals = std::pow(10, precision);
    return (std::round(number * decimals)) / decimals;
}

Route::Route(EAV* vehicle): 
    batteryFeasible(true),
    capacityFeasible(true),
    timeFeasible(true),
    policy(ChargingPolicy::MINIMAL),
    adaptiveCharging(true),
    vehicle(vehicle){}

int Route::getLoad(int i) {
    return (i == 0) ? 0 : getLoad(i - 1) + path[i]->load;
}

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

    FTS[n-1] = std::max(0.0, vehicle->end_time - start_of_service_times[n-1]);
    for (int i = n - 2; i != -1; i--) {
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
        if (path[j]->isDestination() && node_indices[inst.getRequest(path[j])->origin] < i) {
            ride_time = getRideTime(node_indices[inst.getRequest(path[j])->origin]);
            max_travel_time =inst.getRequest(path[j])->origin->maximum_travel_time;
        }

        double time_slack = cumulWaitingTime +
            std::max(0.0, std::min((j == path.size() - 1) ? vehicle->end_time : path[j]->latest - start_of_service_times[j], max_travel_time - ride_time));

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
        
        cumulWaitingTime += waiting_times[j + 1];
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
   
     return !path[index - 1]->isChargingStation() ? std::max(0.0,path[index]->earliest - (start_of_service_times[index - 1] + path[index - 1]->service_duration + inst.getTravelTime(path[index-1], path[index]))) :
          std::max(0.0,path[index]->earliest - (start_of_service_times[index - 1] + inst.getChargingStation(path[index - 1])->getRequiredTime(battery[index - 1], desiredAmount[inst.getChargingStation(path[index - 1])]) + inst.getTravelTime(path[index-1], path[index])));
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
        waiting_times[i] = getWaitingTime(i);
        computeStartOfServiceTime(i);
    }

    // STEP 7
    for (int j = 1; j < n - 1; j++) {
        if (path[j]->isOrigin()) {

           start_of_service_times[j]+= std::min(get_forward_time_slack(j),
               std::accumulate(waiting_times.begin() + j + 1, waiting_times.end() - 1, 0.0));

            // STEP 7 (c):
           for (int i = j + 1; i < n; i++) {
               waiting_times[i] = getWaitingTime(i);
               computeStartOfServiceTime(i);
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
    requests.clear();
    node_indices.clear();
    desiredAmount.clear();
    for (int i = 0; i < n; i++) {
        node_indices[path[i]] = i;
        computeLoad(i);
        computeBatteryLevel(i);
        if (adaptiveCharging) computeChargingTime(i);
        if (path[i]->isOrigin()) requests.push_back(inst.getRequest(path[i]));
    }
    BasicScheduling();
    LazyScheduling();
    storeNaturalSequences();
    computeTotalCost(debug);
}

void Route::computeLoad(int i) {
    loads[i] = (i == 0) ? 0 : loads[i-1] + path[i]->load;
}

void Route::computeStartOfServiceTime(int i)
{
    start_of_service_times[i]=!path[i-1]->isChargingStation()? std::max(path[i]->earliest,
            start_of_service_times[i - 1] + path[i-1]->service_duration + inst.getTravelTime(path[i - 1], path[i])):
        std::max(path[i]->earliest,
            start_of_service_times[i - 1] + inst.getChargingStation(path[i-1])->getRequiredTime(battery[i-1], desiredAmount[inst.getChargingStation(path[i-1])]) + inst.getTravelTime(path[i-1], path[i]));
     
}

void Route::deleteRedundantChargingStations()
{ 
    bool routeChanged = false;
    for (std::pair<CStation*,double> pair:desiredAmount){
        Node* cs_node = inst.nodes[pair.first->id - 1];
        if (node_indices.contains(cs_node)) {
            int index = node_indices[cs_node];
            double charging_time = pair.first->getRequiredTime(battery[index],desiredAmount[pair.first]);
            if (dbl_round(charging_time,1)==0.0) {
              removeNode(index);
              node_indices.erase(cs_node);
              std::for_each(path.begin() + index, path.end(),
                  [this](Node* node) {node_indices[node]--; }
              );
              inst.operationsSaved++;
              routeChanged = true;
            }
        }
    }
    if (routeChanged) { 
        updateMetrics();
        inst.operationsSaved--;
    }

}

void Route::computeBatteryLevel(int i) {
    if (i == 0) battery[i] = vehicle->initial_battery;
    else {
        battery[i] = battery[i-1] - inst.getFuelConsumption(path[i-1], path[i]);
        if (path[i-1]->isChargingStation()) {
            CStation* s = inst.getChargingStation(path[i-1]);
            battery[i] += s->getRequiredTime(battery[i-1],desiredAmount[s]) * s->recharge_rate;
        }
    }
}

void Route::computeTotalCost(bool debug) {
    int n = path.size();
    travel_distance = excess_ride_time = 0.0;

    for (int i = 0; i < n-1; i++) {
        travel_distance += inst.getTravelTime(path[i], path[i + 1]);
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
        if (dbl_round(start_of_service_times[i] - path[i]->latest, 3)>0.0 || dbl_round(getRideTime(i)-path[i]->maximum_travel_time, 3)>0.0) {
            timeFeasible = false;
            if (debug) {
                std::cout << "Time Window:" << dbl_round(start_of_service_times[i] - path[i]->latest, 3) << "\n";
                std::cout << "Ride Time:" << getRideTime(i) << "\n";
                std::cout << "Breakpoint:" << "\n";
            }
            return;
        }
        
        if (path[i]->isOrigin()) excess_ride_time += getRideTime(i) - inst.getTravelTime(path[i], inst.getRequest(path[i])->destination);
        
    }

    if (loads[n-1] > vehicle->capacity) {
        capacityFeasible = false;
        if (debug) {
            std::cout << "Capacity Infeasible" << std::endl;
        }
        return;
    }
    if (dbl_round(battery[n-1], 3) < dbl_round(vehicle->minBatteryUponReturn, 3)) {
        batteryFeasible = false;
        if (debug) {
            std::cout << "Return Battery Infeasible: " << dbl_round(battery[n-1] - vehicle->minBatteryUponReturn, 3) << "\n";
        }
        return;
    }
    if (dbl_round(vehicle->end_time-start_of_service_times[n-1], 3)<0.0) {
        timeFeasible = false;
        if (debug) {
            std::cout << "Time Window:" << dbl_round(start_of_service_times[n-1] - vehicle->end_time, 3) << "\n";
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
    if (isEmpty())  path.push_back(node); 
    else {
        if (index < 1)
            perror("Error! Can not insert a node before the starting depot");
        else if (index > path.size()) perror("Error! Can not insert a node after the ending depot");
        else { 
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
        double desired_amount = 0.0;
        if (policy == ChargingPolicy::FULL)  desired_amount = vehicle->total_battery;
        else {
            //Calculate the fuel to be added in order to reach a charging station or the ending depot.
            Node* current_node = path[nodePosition];
            for (size_t x = nodePosition + 1; x < path.size(); x++) {
                desired_amount += inst.getFuelConsumption(current_node, path[x]);
                if (path[x]->isChargingStation()) break;
                current_node = path[x];
            }
            if (current_node->isEndingDepot()) desired_amount += vehicle->minBatteryUponReturn;
            desiredAmount[s]=std::min(desired_amount, vehicle->total_battery);
            
        }
      
    }
}

std::pair<size_t, size_t> Route::getRequestPosition(const Request* r) {
    std::pair<size_t,size_t> positions;
    if (node_indices.contains(r->origin) && node_indices.contains(r->destination)) {
        positions.first = node_indices[r->origin];
        positions.second = node_indices[r->destination];
    }
    else perror("Error! This request is not served during this route");
    return positions;
}

CStation* Route::findBestChargingStationAfter(int i,std::unordered_map<CStation*,unsigned int>& stationVisits) {
    //Find all the reachable stations from the zero-load node
    std::random_device rd;
    RandLib randlib(rd());
    std::vector<CStation*> reachable_stations;
    reachable_stations.reserve(inst.charging_stations.size());
    std::copy_if(inst.charging_stations.begin(),
        inst.charging_stations.end(),
        std::back_inserter(reachable_stations),
        [&stationVisits,i,this](CStation* station) {return stationVisits[station] < inst.maxVisitsPerStation &&
        !desiredAmount.contains(station) &&
        battery[i] - inst.getFuelConsumption(path[i], inst.nodes[station->id - 1]) >= 0.0; }
    );
    if (!reachable_stations.empty())
    {
        //Find the charging station with the minimum distance that can be reached from the node.
        CStation* min_s = reachable_stations[0];
        double min_score = inst.dischargeRate*getAddedDistance(inst.nodes[min_s->id - 1], i);

        for (std::vector<CStation*>::iterator it = reachable_stations.begin(); it!=reachable_stations.end(); ++it)
        {
            
            double current_score = inst.dischargeRate*getAddedDistance(inst.nodes[(*it)->id - 1], i);
            if (min_score > current_score) {
                min_s = *it;
                min_score = current_score;
            }
        }
        return min_s;
    }
    return nullptr;
    
}

void Route::removeNode(int index){
    if (index > 0 && index < path.size()) {
        if (!adaptiveCharging && path[index]->isChargingStation()) {
            desiredAmount.erase(inst.getChargingStation(path[index]));
        }
        path.erase(path.begin() + index);
    }
}

void Route::removeRequest(Request* request)
{
    std::pair < size_t, size_t > indices= getRequestPosition(request);
    if (path[indices.first]->id != request->origin->id || path[indices.second]->id != request->destination->id)
        std::cout << "Error in removal." << "\n";
    removeNode(indices.first);
    removeNode(indices.second-1);
}

bool Route::isInsertionCapacityFeasible(Request* request, int i, int j) const {
    if (j < i || i < 0 || j < 0 || i + 1 >= path.size() || j + 1 >= path.size()) return false;
    else { 
        for (size_t x = i; x <= j; x++) {
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
            for (const auto& [station,amount]:desiredAmount) {
                int index = node_indices[inst.nodes[station->id - 1]];
                if (index <= i && index > closest_left) closest_left = index;
                if (index > j && index < closest_right) closest_right = index;
            }
            double limit;
            if (!closest_left || policy == ChargingPolicy::FULL) { 
                limit = (closest_right == path.size() - 1) ?vehicle->minBatteryUponReturn:0.0;
                return battery[closest_right] - getAddedDistance(request, i, j)*inst.dischargeRate >= limit; 
            }
            else {
                CStation* s = inst.getChargingStation(path[closest_left]);
                return std::max(0.0,desiredAmount[s]-battery[closest_left])
                    + getAddedDistance(request, i, j)*inst.dischargeRate <= vehicle->total_battery;
            }
        }
        else {
            double battery_after_insertion;
            double added_battery_consumption = getAddedDistance(request, i, j)*inst.dischargeRate;
            for(const auto& [station,amount] : desiredAmount) {
                int index = node_indices[inst.nodes[station->id - 1]];
                if (index > j) {
                    battery_after_insertion = battery[index] - added_battery_consumption;
                    if (battery_after_insertion < 0.0) return false;
                }
                
            }
            battery_after_insertion = battery.back() - added_battery_consumption;
            return battery_after_insertion >= vehicle->minBatteryUponReturn;
        }
    }
}

bool Route::isInsertionTimeFeasible(Request* request, int i, int j)
{
    //Check if a charging station is located before the request.
    double new_bos_i = start_of_service_times[i];
    int closest = 0;
    for (const auto& [station,amount]:desiredAmount) {
         int csIndex = node_indices[inst.nodes[station->id - 1]];
         if (csIndex > closest && csIndex <=i) closest = csIndex;
    }

    if (closest) {
        CStation* cs = inst.getChargingStation(path[closest]);
        double added_charging_time = cs->getRequiredTime(battery[closest]+getAddedDistance(request,i,j)*inst.dischargeRate,desiredAmount[cs])-
            cs->getRequiredTime(battery[closest],desiredAmount[cs]);
        if (added_charging_time > FTS[closest]) return false;
        new_bos_i += std::max(0.0, added_charging_time - std::accumulate(waiting_times.begin() + closest + 1, waiting_times.begin() + i + 1, 0.0));
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


