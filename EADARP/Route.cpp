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
    vehicle(vehicle),
    travel_distance(0.0)
{}

void Route::BasicScheduling() {
    int n = path.size();
    base_schedule.clear();
    late_schedule.clear();
    base_schedule.resize(n);
    late_schedule.resize(n);
    FTS.clear();
    FTS.resize(n);
    base_waiting_times.clear();
    late_waiting_times.clear();
    base_waiting_times.resize(n);
    late_waiting_times.resize(n);


    base_schedule[0]=late_schedule[0]=vehicle->start_time;
    base_waiting_times[0] = late_waiting_times[0] = 0.0;
    for (int i = 1; i < n; i++) {
        computeStartOfServiceTime(i);
        late_schedule[i] = base_schedule[i];
        if (dbl_round(late_schedule[i] - path[i]->latest, 3) > 0.0) {
            timeFeasible = false;
            return;
        }
        computeWaitingTime(i);
        late_waiting_times[i] = base_waiting_times[i];
    }

    FTS[n - 1] = vehicle->end_time - base_schedule[n - 1];
    for (int i = n - 2; i != -1; i--)
        FTS[i] = std::min(path[i]->latest - base_schedule[i],
            base_waiting_times[i + 1] + FTS[i + 1]); 
}

void Route::storeNaturalSequences()
{
    natural_sequences.clear();
    for (int i = 1; i < path.size(); i++) {
        if (loads[i]) {
            int first = i;
            while (loads[i]) i++;
            natural_sequences.emplace_back(first,i);
        }
    }
}

double Route::getForwardTimeSlack(int i)
{
    double min_time_slack = DBL_MAX;
    double cumulWaitingTime = 0.0;
    for (int j = i; j < path.size(); j++) {
        double ride_time = 0.0, max_travel_time=path[j]->maximum_travel_time;
        
        if (path[j]->isDestination()) {
            int origin_index = node_indices[inst.getRequest(path[j])->origin];
            if (origin_index < i) {
                ride_time = getRideTime(origin_index);
                max_travel_time = path[origin_index]->maximum_travel_time;
            }
        }
            

        double time_slack = cumulWaitingTime +
            std::max(0.0,std::min((j == path.size() - 1 ? vehicle->end_time : path[j]->latest) -late_schedule[j],max_travel_time - ride_time));

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
        if (j != path.size() - 1) cumulWaitingTime += late_waiting_times[j + 1];
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
            time_slack = cumul_waiting_time + std::max(0.0, (j == path.size() - 1) ? vehicle->end_time : path[j]->latest -late_schedule[j]);

        if (time_slack < min_time_slack)
            min_time_slack = time_slack;
        
    }

    return min_time_slack;
}

void Route::computeWaitingTime(int index,bool base) {
    double serviceDuration = (path[index - 1]->isChargingStation()) ? inst.getChargingStation(path[index - 1])->getRequiredTime(battery[index - 1], departureBatteryLevel[inst.getChargingStation(path[index - 1])]) : path[index - 1]->service_duration;
    base?
    base_waiting_times[index]= std::max(0.0, path[index]->earliest - 
        (base_schedule[index - 1] + serviceDuration + inst.getTravelTime(path[index - 1], path[index])))
    :
    late_waiting_times[index] = std::max(0.0, path[index]->earliest - 
        (late_schedule[index - 1] +serviceDuration+ inst.getTravelTime(path[index - 1], path[index])));
    
}

double Route::getRideTime(int index) {
    return path[index]->isOrigin() ?
        late_schedule[node_indices[inst.getRequest(path[index])->destination]]-
        (late_schedule[index]+path[index]->service_duration): 0.0;
}

void Route::LazyScheduling() {
    size_t n = path.size();
    late_schedule[0] = vehicle->start_time+std::min(getForwardTimeSlack(0),
        std::accumulate(late_waiting_times.begin()+1,late_waiting_times.end()-1,0.0));

    for (int i = 1; i < n; i++) {
        computeStartOfServiceTime(i,false);
        computeWaitingTime(i, false);
    }

    // STEP 7
    for (int j = 1; j < n - 1; j++) {
        if (path[j]->isOrigin()) {

           late_schedule[j]+= std::min(getForwardTimeSlack(j),
               std::accumulate(late_waiting_times.begin() + j+1, late_waiting_times.end()-1, 0.0));

            // STEP 7 (c):
           for (int i = j + 1; i < n; i++) {
               computeStartOfServiceTime(i,false);
               computeWaitingTime(i, false);
           }
        }
    }

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
        if (path[i]->isChargingStation()) computeChargingTime(i);
        computeLoad(i);
        if (loads[i] > vehicle->capacity) { 
            capacityFeasible = false;
            return; 
        }
        computeBatteryLevel(i);
        if (dbl_round(battery[i], 3)<vehicle->minBatteryUponReturn*(i==n-1)) {
            batteryFeasible = false;
            return;
        }
    }
    BasicScheduling();
    if (!timeFeasible) return;
    LazyScheduling();
    computeExcessRideTime();
    if (!timeFeasible) return;
    storeNaturalSequences();
   
}

void Route::computeLoad(int i) {
    loads[i] = loads[i-1] + path[i]->load;
}

void Route::computeStartOfServiceTime(int i,bool base)
{
    double serviceDuration = (path[i - 1]->isChargingStation()) ? inst.getChargingStation(path[i - 1])->getRequiredTime(battery[i - 1], departureBatteryLevel[inst.getChargingStation(path[i - 1])]) : path[i - 1]->service_duration;
    base?
        base_schedule[i] = std::max(path[i]->earliest,
            base_schedule[i - 1] + serviceDuration + inst.getTravelTime(path[i - 1], path[i]))
           
    :
        late_schedule[i] =
            std::max(path[i]->earliest,
                late_schedule[i - 1] + serviceDuration + inst.getTravelTime(path[i - 1], path[i]));
    
}

bool Route::deleteRedundantChargingStations()
{ 
    bool routeChanged = false;
    for (const auto& [station,amount]:departureBatteryLevel){
        int stationIndex = node_indices[inst.nodes[station->id - 1]];
        if (!station->getRequiredTime(battery[stationIndex], amount)) {
          removeNode(stationIndex);
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

void Route::computeExcessRideTime() {
    size_t n = path.size();
    excess_ride_time = 0.0;

    for (Request* req : requests) {
        int index = node_indices[req->origin];
        double ride_time = getRideTime(index);
        if (dbl_round(ride_time - path[index]->maximum_travel_time, 3) > 0.0) {
            timeFeasible = false;
            return;
        }
        excess_ride_time += ride_time - inst.getTravelTime(req->origin, req->destination);
    }
}

//Inserting node after position i
double Route::getAddedDistance(Node* node, int i) const {
    return inst.getTravelTime(path[i], node)
        + inst.getTravelTime(node, path[i+1])
        - inst.getTravelTime(path[i], path[i+1]);
}

//Inserting origin after i and destination after j.
double Route::getAddedDistance(const Request* request, int i, int j) const {
    if (i == j) return inst.getTravelTime(path[i], request->origin)
            + inst.getTravelTime(request->origin, request->destination)
            + inst.getTravelTime(request->destination, path[i+1])
            - inst.getTravelTime(path[i], path[i+1]);

    else return getAddedDistance(request->origin, i) + getAddedDistance(request->destination, j);
}

//This function inserts a node in a route before the specified index
void Route::insertNode(Node* node, int index)
{
    std::for_each(path.begin() + index, path.end(), [&](Node* node) {node_indices[node]++; });
    travel_distance += getAddedDistance(node,index-1);
    path.insert(path.begin() + index, node);
    node_indices.try_emplace(node,index);
    if (node->isChargingStation()) departureBatteryLevel.try_emplace(inst.getChargingStation(node),0.0);
    
}

void Route::insertRequest(Request* r, int i, int j) {
    insertNode(r->origin, i);
    insertNode(r->destination, j + 1);
    requests.push_back(r);
}

void Route::computeChargingTime(int stationPosition) {
    CStation* station = inst.getChargingStation(path[stationPosition]);
    double desired_amount = 0.0;
    //Calculate the fuel to be added in order to reach a charging station or the ending depot.
    Node* current_node = path[stationPosition];
    for (size_t x = stationPosition + 1; x < path.size(); x++) {
        desired_amount += inst.getFuelConsumption(current_node, path[x]);
        if (path[x]->isChargingStation()) break;
        current_node = path[x];
    }
    departureBatteryLevel.insert_or_assign(station,std::min(desired_amount+current_node->isEndingDepot()*vehicle->minBatteryUponReturn, vehicle->total_battery));
}

std::pair<size_t, size_t> Route::getRequestPosition(const Request* r) {
    return { node_indices[r->origin] , node_indices[r->destination] };
}

CStation* Route::findBestChargingStationAfter(int i,std::unordered_map<CStation*,unsigned int>& stationVisits) {
    //Find all the reachable stations from the zero-load node
    std::vector<CStation*> reachable_stations;
    reachable_stations.reserve(inst.charging_stations.size());
    std::copy_if(inst.charging_stations.begin(),
        inst.charging_stations.end(),
        std::back_inserter(reachable_stations),
        [&](CStation* station) {return stationVisits[station] < inst.maxVisitsPerStation &&
        !departureBatteryLevel.contains(station) &&
        battery[i] - inst.getFuelConsumption(path[i], inst.nodes[station->id - 1]) >= 0.0; }
    );
   
    //Find the charging station with the minimum distance that can be reached from the node.
    return reachable_stations.empty()?nullptr:*std::min_element(reachable_stations.begin(), reachable_stations.end(), 
        [&](const CStation* cs1, const CStation* cs2) 
        {
            return  getAddedDistance(inst.nodes[cs1->id - 1], i) <
                getAddedDistance(inst.nodes[cs2->id - 1], i);
        }
    );
    
}

void Route::removeNode(int index){
   
    node_indices.erase(path[index]);
    travel_distance-=inst.getTravelTime(path[index - 1], path[index])+
         inst.getTravelTime(path[index], path[index + 1])-
         inst.getTravelTime(path[index - 1], path[index + 1]);
     departureBatteryLevel.erase(inst.getChargingStation(path[index]));
     path.erase(path.begin() + index);
     std::for_each(path.begin() + index, path.end(),
         [&](Node* node) {node_indices[node]--; }
     );
}

void Route::removeRequest(const Request* request)
{
    removeNode(node_indices[request->origin]);
    removeNode(node_indices[request->destination]);
    requests.erase(std::remove(requests.begin(), requests.end(), request),requests.end());
}

bool Route::isInsertionCapacityFeasible(const Request* request, int i, int j) const {
    
    return std::none_of(path.begin() + i+1, path.begin() + j + 1, [&](Node* node) {
        return loads[node_indices.at(node)] + request->origin->load > vehicle->capacity || 
            node->isChargingStation();
        }) && loads[i] + request->origin->load <= vehicle->capacity;
}

/*
  Checks that the battery level at the assigned charging stations and the depot is above 0.
  Worst case running time: O(|S|), where S the set of available charging stations. 
*/
bool Route::isInsertionBatteryFeasible(const Request* request, int i, int j) {
  
    
     //Spot the important nodes (depots,charging stations) the request lies between.
     int closest_left = 0,closest_right = path.size() - 1;
     for (const auto& [station,amount]:departureBatteryLevel) {
         int index = node_indices[inst.nodes[station->id - 1]];
         if (unsigned(index-closest_left)<=i-closest_left) closest_left = index;
         if (unsigned(index-j-1)< closest_right-j) closest_right = index;
     }
    

     double added_battery_consumption = getAddedDistance(request, i, j) * inst.dischargeRate;
     if (!closest_left) {
         double limit = (closest_right == path.size() - 1) ? vehicle->minBatteryUponReturn : 0.0;
         return battery[closest_right] - added_battery_consumption >= limit;
     }
     return battery[closest_left + 1] + inst.getFuelConsumption(path[closest_left], path[closest_left + 1])
         + added_battery_consumption <= vehicle->total_battery;
         
}

bool Route::isInsertionTimeFeasible(const Request* request, size_t i, size_t j)
{
    //Check if a charging station is located before the request.
    
    double new_bos_i = base_schedule[i];
    double added_consumption = getAddedDistance(request, i, j) * inst.dischargeRate;
    int closest_left = 0;
    std::vector<int> stationsAfterRequest;
    stationsAfterRequest.reserve(inst.charging_stations.size());
    for (const auto& [station, amount] : departureBatteryLevel) {
        int csIndex = node_indices[inst.nodes[station->id - 1]];
        if (unsigned(csIndex - closest_left) <= i - closest_left) closest_left = csIndex;
        if (csIndex > j) stationsAfterRequest.push_back(csIndex);
    }
    std::sort(stationsAfterRequest.begin(), stationsAfterRequest.end());
    double added_time,charging_time_left = 0.0;
    if (closest_left) {
        CStation* cs = inst.getChargingStation(path[closest_left]);
        if (departureBatteryLevel[cs] + added_consumption>vehicle->total_battery) return false;
        charging_time_left = cs->getRequiredTime(battery[closest_left], departureBatteryLevel[cs] + added_consumption);
        if (closest_left < i) {
            added_time = std::max(0.0, base_schedule[closest_left] + charging_time_left +
                inst.getTravelTime(path[closest_left], path[closest_left + 1]) - base_schedule[closest_left + 1]);
            if (added_time > FTS[closest_left+1]) return false;
            new_bos_i += std::max(0.0, added_time - std::accumulate(base_waiting_times.begin() + closest_left + 2, base_waiting_times.begin() + i + 1, 0.0));
        }
    }
  
    double bos_origin = std::max(request->origin->earliest, new_bos_i + (closest_left == i ? charging_time_left : path[i]->service_duration) + inst.getTravelTime(path[i], request->origin));
    if (bos_origin > request->origin->latest) return false;

    double bos_destination;
    if (i != j) {
        added_time = std::max(0.0, bos_origin + request->origin->service_duration + inst.getTravelTime(request->origin, path[i + 1])
            - base_schedule[i + 1]);
        if (added_time > FTS[i + 1]) return false;
        double new_bos_j = base_schedule[j] +
            std::max(0.0, added_time - std::accumulate(base_waiting_times.begin() + i + 2, base_waiting_times.begin() + j + 1, 0.0));
        bos_destination = std::max(request->destination->earliest, new_bos_j + path[j]->service_duration +
            inst.getTravelTime(path[j], request->destination));
    }
    else bos_destination = std::max(request->destination->earliest, bos_origin + request->origin->service_duration +
            inst.getTravelTime(request->origin, request->destination));
    
    if (bos_destination > request->destination->latest) return false;
    added_time = std::max(0.0,
        bos_destination + request->destination->service_duration + inst.getTravelTime(request->destination, path[j + 1]) - base_schedule[j + 1]);

    if (added_time > FTS[j + 1]) return false;

    if (charging_time_left) return true;
    for (int csIndex : stationsAfterRequest) {
        if (battery[csIndex] < added_consumption) return false;
        CStation* cs = inst.getChargingStation(path[csIndex]);
        double charging_time = cs->getRequiredTime(battery[csIndex] - added_consumption, departureBatteryLevel[cs]);
        if (charging_time-cs->getRequiredTime(battery[csIndex],departureBatteryLevel[cs])) {
            double bos_station = base_schedule[csIndex] +
                std::max(0.0, added_time - std::accumulate(base_waiting_times.begin() + j + 2, base_waiting_times.begin() + csIndex + 1, 0.0));
            added_time = std::max(0.0, bos_station + charging_time + inst.getTravelTime(path[csIndex], path[csIndex + 1])
                - base_schedule[csIndex + 1]);
            return added_time <= FTS[csIndex + 1];
        }
    }
    
    return true;
}


