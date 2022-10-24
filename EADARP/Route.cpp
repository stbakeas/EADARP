#include "Route.h"
#include <iostream>
#include "Instance.h"
#include "CStation.h"
#include <numeric>
#include <limits>
#include <algorithm>

Route::Route(EAV* vehicle) {
	this->vehicle = vehicle;
    for (CStation* s : inst.charging_stations) { 
        recharge_times[s] = 0;
        assigned_cs[s] = false;
    }
}

double Route::getEarliestTime(int i) {
    if (i == 0)
        return sequence.at(i)->earliest;
    else
        return std::max(
            sequence.at(i)->earliest,
            getEarliestTime(i - 1) + sequence.at(i-1)->service_duration+ inst.getTravelTime(sequence.at(i-1), sequence.at(i))
        );
}

int Route::getLoad(int i) {
    return (i == 0) ? 0 : getLoad(i - 1) + sequence.at(i)->load;
}

bool Route::isEmpty() {
    return sequence.size() == 0; //Only the origin and destination depots are a part of the route
}

bool Route::hasNoRequests() {
    return sequence.size() <= 2; //Contains only the depot node.
}

bool Route::feasible() {
    return cost != DBL_MAX && cost != -DBL_MAX;
}

void Route::updateMetrics() {
    int n = (int)sequence.size();
    arrival_times.clear();
    arrival_times.resize(n);
    departure_times.clear();
    departure_times.resize(n);
    waiting_times.clear();
    waiting_times.resize(n);
    loads.clear();
    loads.resize(n);
    ml.clear();
    ml.resize(n);
    start_of_service_times.clear();
    start_of_service_times.resize(n);
    ride_times.clear();
    ride_times.resize(n);
    battery.clear();
    battery.resize(n);
    stay_times.clear();
    stay_times.resize(n);
    FTS.clear();
    FTS.resize(n);
    for (int i = 0; i < n; i++) {
        ml.at(i).resize(n);
        computeArrivalTime(i);
        computeStartOfServiceTime(i);
        computeDepartureTime(i);
        computeLoad(i);
        computeWaitingTime(i);
        computeBatteryLevel(i);
        computeStayTime(i);
        if (sequence.at(i)->type==Node::Type::ORIGIN || sequence.at(i)->type == Node::Type::DESTINATION) computeRideTime(i);
        computeForwardTimeSlack(n - i - 1);
    }
    //We need to compute the loads at each node of the route first.
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) computeMaximumLoadBetween(i, j);
    }
   
    
}

void Route::computeLoad(int i) {
    loads.at(i) = (i == 0) ? 0 : loads.at(i - 1) + sequence.at(i)->load;
}

void Route::computeForwardTimeSlack(int i) {
    if (i== sequence.size()-1) FTS.at(i) = waiting_times.at(i) +
        std::max(0.0,std::min(sequence.at(i)->latest-arrival_times.at(i),sequence.at(i)->maximum_travel_time-ride_times.at(i)));
    else FTS.at(i)= waiting_times.at(i) +
        std::min(FTS.at(i+1),std::max(0.0, std::min(sequence.at(i)->latest - arrival_times.at(i), sequence.at(i)->maximum_travel_time - ride_times.at(i))));
}

void Route::computeArrivalTime(int i)
{
    if (i == 0) {
        arrival_times.at(i) = sequence.at(i)->earliest;
    }
    else {
        arrival_times.at(i) = departure_times.at(i - 1) + inst.getTravelTime(sequence.at(i - 1), sequence.at(i));
    }   
}

void Route::computeStartOfServiceTime(int i)
{
    start_of_service_times.at(i) = std::max(arrival_times.at(i), sequence.at(i)->earliest);
}

void Route::computeWaitingTime(int i)
{
    waiting_times.at(i) = start_of_service_times.at(i) - arrival_times.at(i);
}

void Route::computeDepartureTime(int i)
{
    departure_times.at(i) = start_of_service_times.at(i) + sequence.at(i)->service_duration;
    if (sequence.at(i)->type == Node::Type::CHARGING_STATION) departure_times.at(i) += stay_times.at(i);
}

void Route::computeRideTime(int i)
{
    ride_times.at(i) = start_of_service_times.at(node_indices[inst.getRequest(sequence.at(i))->destination]) - departure_times.at(i);
}

void Route::computeStayTime(int i) {
    if (sequence.at(i)->type == Node::Type::CHARGING_STATION) {
        CStation* s = inst.getChargingStation(sequence.at(i));
        stay_times.at(i) = recharge_times[s];
    }
    else stay_times.at(i) = waiting_times.at(i);
}

void Route::computeBatteryLevel(int i) {
    if (i == 0) battery.at(i) = vehicle->battery;
    else {
        battery.at(i) = battery.at(i - 1) - inst.getTravelTime(sequence.at(i - 1), sequence.at(i));
        if (sequence.at(i - 1)->type == Node::Type::CHARGING_STATION) {
            CStation* s = inst.getChargingStation(sequence.at(i - 1));
            battery.at(i) += recharge_times[s] * s->recharge_rate;
        }
        if (sequence.at(i)->type == Node::Type::CHARGING_STATION) {
            CStation* s = inst.getChargingStation(sequence.at(i));
            recharge_times[s] = s->getRequiredTime(battery.at(i), vehicle->battery);
        }

    }
}

void Route::computeMaximumLoadBetween(int i, int j) {
    if (i > j) ml.at(i).at(j) = INT_MAX;
    else {
        int max_load = loads.at(i);
        for (int x =i+1; x <= j; x++)
        {
            if (max_load < loads.at(x)) max_load = loads.at(x);
        }
        ml.at(i).at(j) = max_load;
    }

}

double Route::Evaluate() {
    int size = sequence.size();

    arrival_times.clear();
    arrival_times.resize(size);
    battery.clear();
    battery.resize(size);
    start_of_service_times.clear();
    start_of_service_times.resize(size);
    departure_times.clear();
    departure_times.resize(size);
    waiting_times.clear();
    waiting_times.resize(size);
    FTS.clear();
    FTS.resize(size);
    ride_times.clear();
    ride_times.resize(size);
    loads.clear();
    loads.resize(size);

    double forward_time_slack_at_0;

    node_indices[sequence.at(0)] = 0;

    // STEP 1
    computeBatteryLevel(0);
    departure_times.at(0) = sequence.at(0)->earliest;
    start_of_service_times.at(0) = departure_times.at(0);
    

    // STEP 2
    for (int i = 1; i < sequence.size(); i++) {
        computeLoad(i);
        // Violated vehicle capacity, which is an irreparable violation
        if (loads.at(i) > vehicle->capacity) return DBL_MAX;

        computeBatteryLevel(i);
        if (battery.at(i) < 0) return -DBL_MAX;

        computeArrivalTime(i);
        computeStartOfServiceTime(i);

        //if (start_of_service_times.at(i) > sequence.at(i)->latest) goto STEP8;

        computeWaitingTime(i);
        computeDepartureTime(i);

        node_indices[sequence.at(i)] = i;
    }

    // STEP 3
    for (int i = size-1; i>=0; --i) computeForwardTimeSlack(i);
   
    forward_time_slack_at_0 = FTS.at(0);

    // STEP 4
    departure_times.at(0) = sequence.at(0)->earliest + std::min(
        forward_time_slack_at_0, std::accumulate(waiting_times.begin() + 1, waiting_times.end() - 1, 0.0)
    );

    start_of_service_times.at(0) = departure_times.at(0);

    // STEP 5
    for (int i = 1; i < sequence.size(); i++) {
        computeArrivalTime(i);
        computeStartOfServiceTime(i);
        computeWaitingTime(i);
        computeDepartureTime(i);
    }

    // STEP 6
    for (int i = 1; i < sequence.size() - 1; i++)
        if (sequence.at(i)->isOrigin())
            computeRideTime(i);

    // STEP 7
    for (int j = 1; j < sequence.size() - 1; j++) {
        if (sequence.at(j)->isOrigin()) {
            // STEP 7 (a)
            //computeForwardTimeSlack(j);
            double forward_time_slack = FTS.at(j);

            // STEP 7 (b)
            waiting_times.at(j) += std::min(
                forward_time_slack, std::accumulate(waiting_times.begin() + j + 1, waiting_times.end() - 1, 0.0)
            );

            start_of_service_times.at(j) = arrival_times.at(j) + waiting_times.at(j);
            departure_times.at(j) = start_of_service_times.at(j) + sequence.at(j)->service_duration;

            // STEP 7 (c):
            for (int i = j + 1; i < sequence.size(); i++) {
                computeArrivalTime(i);
                computeStartOfServiceTime(i);
                computeWaitingTime(i);
                computeDepartureTime(i);
            }

            // STEP 7 (d):
            for (int i = j + 1; i < sequence.size() - 1; i++)
                if (sequence.at(i)->isDestination())
                    computeRideTime(node_indices[inst.getRequest(sequence.at(i))->origin]);
        }
    }

STEP8:
    cost = 0.0;
    time_window_violation = 0.0;
    max_ride_time_violation = 0.0;
    returned_battery_violation = 0.0;

    for (int i = 1; i < sequence.size(); i++) {
        cost += inst.getTravelTime(sequence.at(i - 1), sequence.at(i));
        time_window_violation += std::max(0.0, start_of_service_times.at(i) - sequence.at(i)->latest);

        if (sequence.at(i)->isOrigin())
            max_ride_time_violation += std::max(0.0, ride_times.at(i) - sequence.at(i)->maximum_travel_time);
    }
    returned_battery_violation = std::max(0.0, vehicle->battery_return_percentage * vehicle->battery - battery.back());

    return cost + time_window_violation + max_ride_time_violation + returned_battery_violation;
}

//Inserting node after position i
double Route::getAddedDistance(Node* node, int i, bool timeMode) {
    double dist = inst.getTravelTime(sequence.at(i), node)
        + inst.getTravelTime(node, sequence.at(i + 1))
        - inst.getTravelTime(sequence.at(i), sequence.at(i + 1));

    if (timeMode) {
        double arrival_time = departure_times.at(i) + inst.getTravelTime(sequence.at(i), node);
        double waiting_time = std::max(0.0, node->earliest - arrival_time);
        dist += waiting_time + node->service_duration;
    }
    return dist;
}

//Inserting origin after i and destination after j.
double Route::getAddedDistance(Request* request, int i, int j, bool timeMode) {
    if (i == j) {
        double dist = inst.getTravelTime(sequence.at(i), request->origin)
            + inst.getTravelTime(request->origin, request->destination)
            + inst.getTravelTime(request->destination, sequence.at(i + 1))
            - inst.getTravelTime(sequence.at(i), sequence.at(i + 1));
        if (timeMode) {
            double arrival_time_origin = departure_times.at(i) + inst.getTravelTime(sequence.at(i), request->origin);
            double waiting_time_origin = std::max(0.0, request->origin->earliest - arrival_time_origin);
            double arrival_time_dest = arrival_time_origin + waiting_time_origin + request->origin->service_duration +
                inst.getTravelTime(request->origin, request->destination);
            double waiting_time_dest = std::max(0.0, request->destination->earliest - arrival_time_dest);
            dist += waiting_time_origin + waiting_time_dest + 2 * request->origin->service_duration;
        }
        return dist;
    }
    else return getAddedDistance(request->origin, i, timeMode) + getAddedDistance(request->destination, j, timeMode);
}

double Route::getInsertionCost(Request* request, int i, int j,bool myMode) {
   
    
        //Added Distance
        double distanceCost = getAddedDistance(request, i, j);
        
        //Calculate returned battery violation
        double desiredBattery = vehicle->battery * vehicle->battery_return_percentage;
        double returnedBatteryViolation = std::max(0.0, desiredBattery - battery.back() + getAddedDistance(request, i, j));

        if (isInsertionTimeFeasible(request, i, j)) {
            return distanceCost + returnedBatteryViolation;
        }
        else {
            double timeWindowViolation = 0.0, rideTimeViolation = 0.0;

#pragma region Time Window Violation
            if (i == j) {
                //Violation of the time windows of all nodes exceeding origin and destination
                for (size_t x = i + 1; x < sequence.size(); x++)
                {
                    double new_arr_time = arrival_times.at(x) +
                        std::max(0.0, getAddedDistance(request, i, j, true)
                            - std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + x, 0.0));
                    timeWindowViolation += std::max(0.0, new_arr_time - sequence.at(x)->latest);
                }
                //Violation of the time window of the request.
                double w_origin = std::max(0.0, request->origin->earliest - departure_times.at(i) - inst.getTravelTime(sequence.at(i), request->origin));
                double w_destination = std::max(0.0, request->destination->earliest - (departure_times.at(i) + inst.getTravelTime(sequence.at(i), request->origin) + w_origin + request->origin->service_duration
                    + inst.getTravelTime(request->origin, request->destination)));

                //Origin Violation
                timeWindowViolation += std::max(0.0, departure_times.at(i)
                    + inst.getTravelTime(sequence.at(i), request->origin)
                    + w_origin
                    + request->origin->service_duration
                    - request->origin->latest);

                //Destination Violation
                timeWindowViolation += std::max(0.0, departure_times.at(i) + inst.getTravelTime(sequence.at(i), request->origin) + w_origin + request->origin->service_duration
                    + inst.getTravelTime(request->origin, request->destination) + w_destination + request->destination->service_duration - request->destination->latest);

            }
            else {
                //Violation of the time windows of all nodes exceeding origin and destination
                for (size_t x = i + 1; x < j; x++)
                {
                    double new_arr_time = arrival_times.at(x) +
                        std::max(0.0, getAddedDistance(request->origin, i, true)
                            - std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + x, 0.0));
                    timeWindowViolation += std::max(0.0, new_arr_time - sequence.at(x)->latest);
                }
                double w_origin = std::max(0.0, request->origin->earliest - departure_times.at(i) - inst.getTravelTime(sequence.at(i), request->origin));

                for (size_t x = j; x < sequence.size(); x++)
                {
                    double new_arr_time = arrival_times.at(x) +
                        std::max(0.0, getAddedDistance(request, i, j, true)
                            - std::accumulate(waiting_times.begin() + j, waiting_times.begin() + x, 0.0));
                    timeWindowViolation += std::max(0.0, new_arr_time - sequence.at(x)->latest);
                }

                //Violation of the time window of the request.
                w_origin = std::max(0.0, request->origin->earliest - departure_times.at(i) - inst.getTravelTime(sequence.at(i), request->origin));
                //Origin Violation
                timeWindowViolation += std::max(0.0, departure_times.at(i)
                    + inst.getTravelTime(sequence.at(i), request->origin)
                    + w_origin
                    + request->origin->service_duration
                    - request->origin->latest);


                //Destination violation
                //Find new arrival time at the node before the destination node
                double w_destination = std::max(0.0, request->destination->earliest - (departure_times.at(j) + inst.getTravelTime(sequence.at(j), request->destination)));
                timeWindowViolation += std::max(0.0, departure_times.at(j) + inst.getTravelTime(sequence.at(j), request->destination) + w_destination +
                    request->destination->service_duration - request->destination->latest);
            }
#pragma endregion

#pragma region Ride Time Violation
            if (i == j) {
                std::vector<Node*> origins;
                //1. Find the requests whose ride times are affected by the insertion
                for (size_t x = 0; x <= i; x++)
                {
                    if (sequence.at(x)->type == Node::Type::ORIGIN) origins.push_back(sequence.at(x));
                    if (sequence.at(x)->type == Node::Type::DESTINATION) {
                        Node* origin = inst.getRequest(sequence.at(x))->origin;
                        origins.erase(std::remove(origins.begin(), origins.end(), origin), origins.end());
                    }
                }

                //2. Calculate the ride time violations of the requests from step 1.
                for (Node* node : origins)
                {
                    for (size_t x = i + 1; x < sequence.size(); x++)
                    {
                        if (inst.getRequest(node)->destination == sequence.at(x)) {
                            double new_dest_arr_time = arrival_times.at(x) +
                                std::max(0.0, getAddedDistance(request, i, j, true) -
                                    std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + x, 0.0));
                            double ride_time = std::max(new_dest_arr_time, sequence.at(x)->earliest) - departure_times.at(node_indices[inst.getRequest(sequence.at(x))->origin]);
                            rideTimeViolation += std::max(0.0, ride_time - sequence.at(x)->maximum_travel_time);
                            break;
                        }
                    }
                }
            }
            else {
                std::vector<Node*> origins_before_i, origins_between_i_and_j;
                //1. Find the requests whose ride times are affected by the insertion of the origin of the new request
                for (size_t x = 0; x <= i; x++)
                {
                    if (sequence.at(x)->type == Node::Type::ORIGIN) origins_before_i.push_back(sequence.at(x));
                    if (sequence.at(x)->type == Node::Type::DESTINATION) {
                        Node* origin = inst.getRequest(sequence[x])->origin;
                        origins_before_i.erase(std::remove(origins_before_i.begin(), origins_before_i.end(), origin),
                            origins_before_i.end());
                    }
                }

                for (size_t x = i + 1; x <= j; x++)
                {
                    if (sequence.at(x)->type == Node::Type::ORIGIN) origins_between_i_and_j.push_back(sequence.at(x));
                    if (sequence.at(x)->type == Node::Type::DESTINATION) {
                        Node* origin = inst.getRequest(sequence[x])->origin;
                        origins_between_i_and_j.erase(std::remove(origins_between_i_and_j.begin(), origins_between_i_and_j.end(), origin),
                            origins_between_i_and_j.end());
                    }
                }

                for (Node* o : origins_before_i) {
                    for (size_t x = i + 1; x <= j; x++)
                    {
                        if (inst.getRequest(o)->destination == sequence.at(x)) {
                            double new_dest_arr_time = departure_times.at(x) +
                                std::max(0.0, getAddedDistance(o, i, true) -
                                    std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + x, 0.0));
                            double ride_time = std::max(new_dest_arr_time, sequence.at(x)->earliest) - departure_times.at(node_indices[inst.getRequest(sequence.at(x))->origin]);
                            rideTimeViolation += std::max(0.0, ride_time - sequence.at(x)->maximum_travel_time);
                            break;
                        }
                    }
                    for (size_t x = j + 1; x < sequence.size(); x++) {
                        if (inst.getRequest(o)->destination == sequence.at(x)) {
                            //FIX THIS FORMULA IT IS WRONG!!
                            double new_dest_arr_time = departure_times.at(x) +
                                std::max(0.0, getAddedDistance(request, i, j, true) -
                                    std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + x, 0.0));
                            double ride_time = std::max(new_dest_arr_time, sequence.at(x)->earliest) - departure_times.at(node_indices[inst.getRequest(sequence.at(x))->origin]);
                            rideTimeViolation += std::max(0.0, ride_time - sequence.at(x)->maximum_travel_time);
                            break;
                        }
                    }
                }

                for (Node* o : origins_between_i_and_j)
                {
                    for (size_t x = j + 1; x < sequence.size(); x++)
                    {
                        if (inst.getRequest(o)->destination == sequence.at(x)) {
                            double new_dest_arr_time = departure_times.at(x) +
                                std::max(0.0, getAddedDistance(inst.getRequest(o)->destination, j, true) -
                                    std::accumulate(waiting_times.begin() + j + 1, waiting_times.begin() + x, 0.0));
                            double ride_time = std::max(new_dest_arr_time, sequence.at(x)->earliest) - departure_times.at(node_indices[inst.getRequest(sequence.at(x))->origin]);
                            rideTimeViolation += std::max(0.0, ride_time - sequence.at(x)->maximum_travel_time);
                            break;
                        }

                    }
                }







            }

#pragma endregion


            double violationCost = timeWindowViolation + rideTimeViolation + returnedBatteryViolation;

            return distanceCost + violationCost;

        }
    
  

    
        
}

double Route::getDuration()
{
    return start_of_service_times.back() - start_of_service_times.front();
}

//This function inserts a node in a route before the specified index
void Route::insertNode(Node* node, int index)
{
    if (isEmpty()) sequence.push_back(node);
    else {
        if (index < 1)
            perror("Error! Can not insert a node before the starting depot");
        else if (index > sequence.size()) perror("Error! Can not insert a node after the ending depot");
        else {
            sequence.insert(sequence.begin() + index, node);
            node_indices[sequence.at(index)]=index;
        }
    }
   
}

//Inserts a charging station before the specified index
void Route::insertChargingStation(CStation* s, int index, double desired_amount) {
    insertNode(inst.nodes.at(s->id-1), index);
    assigned_cs[s] = true;
    double battery_upon_arrival = battery.at(index-1)-inst.getTravelTime(sequence.at(index-1),inst.nodes.at(s->id-1));
    if (sequence.at(index - 1)->type == Node::Type::CHARGING_STATION) {
        CStation* station = inst.getChargingStation(sequence.at(index - 1));
        battery_upon_arrival+= recharge_times[station] * station->recharge_rate;
       
    }

    recharge_times[s] = s->getRequiredTime(battery_upon_arrival, desired_amount);
}

void Route::removeChargingStation(CStation* s, int index) {
    if (sequence.at(index)->type == Node::Type::CHARGING_STATION)
    {
        removeNode(index);
        recharge_times[s] = 0;
        assigned_cs[s] = false;
    }
}

CStation* Route::findMinimumDetourReachableStationAfter(int i) {
    //Find all the reachable stations from the zero-load node
    Node* zero_load_node = sequence.at(i);
    std::vector<CStation*> reachable_stations;
    for (CStation* s : inst.charging_stations) {
        if (!assigned_cs[s]) {
            if (battery.at(i) -
                inst.getTravelTime(zero_load_node, inst.nodes.at(s->id - 1))
                >= 0) reachable_stations.push_back(s);
        }
    }
    if (!reachable_stations.empty())
    {
        //Find the charging station with the minimum distance that can be reached from the node.
        CStation* min_s = reachable_stations.at(0);
        double min_dist = inst.getTravelTime(zero_load_node, inst.nodes.at(min_s->id - 1)) +
            inst.getTravelTime(inst.nodes.at(min_s->id - 1), sequence.at(i + 1));
        for (size_t x = 1; x < reachable_stations.size(); x++)
        {
            CStation* current_station = reachable_stations.at(x);
            double current_dist = inst.getTravelTime(zero_load_node, inst.nodes.at(current_station->id - 1)) +
                inst.getTravelTime(inst.nodes.at(current_station->id - 1), sequence.at(i + 1));
            if (min_dist > current_dist) {
                min_s = current_station;
                min_dist = current_dist;
            }
        }
        return min_s;
    }
    return nullptr;
    
}

void Route::removeNode(int index)
{
    if (index > 0 && index < sequence.size() - 1) {
        // Recalculate route's total cost
        cost = cost - inst.getTravelTime(sequence[index - 1], sequence[index])
            - inst.getTravelTime(sequence[index], sequence[index + 1])
            + inst.getTravelTime(sequence[index - 1], sequence[index + 1]);
        std::unordered_map<Node*, int>::iterator it = node_indices.find(sequence.at(index));
        node_indices.erase(it);
        sequence.erase(sequence.begin() + index);
    }
}

void Route::removeRequest(Request* request)
{
    int pickup_index, delivery_index;

    for (int i = sequence.size() - 2; i > 0; i--) {
        if (sequence.at(i) == request->origin) {
            pickup_index = i;
            break;
        }
        else if (sequence.at(i) == request->destination) {
            delivery_index = i;
        }
    }

    removeNode(delivery_index);
    removeNode(pickup_index);
}

//We assume the origin and destination of the request are inserted after indices i and j respectively
bool Route::isInsertionTimeFeasible(Request* request, int i, int j) {
    double arrival_time = departure_times.at(i) + inst.getTravelTime(sequence.at(i), request->origin);
    double waiting_time_origin = std::max(0.0, request->origin->earliest - arrival_time);
    if (j < i || i<0 || j<0 || i+1 >= sequence.size() || j+1 >= sequence.size()) return false;
    else if (i == j) {
        return (departure_times.at(i) + inst.getTravelTime(sequence.at(i), request->origin) + waiting_time_origin +
            request->origin->service_duration + inst.getTravelTime(request->origin, request->destination) <=
            request->destination->latest)
            &&
            getAddedDistance(request,i,j,true)<= FTS.at(i + 1);

    }
    else {
        double origin_detour = getAddedDistance(request->origin, i, true);
        double new_dep_time = departure_times.at(j) + std::max(0.0,origin_detour-std::accumulate(waiting_times.begin() + i + 1, waiting_times.begin() + j, 0.0));
        return new_dep_time + inst.getTravelTime(sequence.at(j), request->destination) <= request->destination->latest &&
            origin_detour <= FTS.at(i + 1) &&
            getAddedDistance(request,i,j,true) <= FTS.at(j + 1);
    }
}

bool Route::isInsertionCapacityFeasible(Request* request, int i, int j) {
    if (j < i || i < 0 || j < 0 || i + 1 >= sequence.size() || j + 1 >= sequence.size()) return false;
    else { 
        return request->origin->load + ml.at(i).at(j) <= vehicle->capacity; 
    }
}

bool Route::isInsertionBatteryFeasible(Request* request, int i, int j) {
    if (j < i || i < 0 || j < 0 || i + 1 >= sequence.size() || j + 1 >= sequence.size()) return false;
    else if (i == j) {
        if (battery.at(i) - inst.getTravelTime(sequence.at(i), request->origin) -
            inst.getTravelTime(request->origin, request->destination) <= 0) return false;
        double extra_distance = getAddedDistance(request, i, j);
        for (int x = i+1; x < sequence.size(); x++)
        {
            if (battery[x] - extra_distance <= 0) return false;
        }
        return true;
    }
    else {
        if (battery.at(i) - inst.getTravelTime(sequence.at(i), request->origin) <= 0) return false;
        double extra_distance_origin = getAddedDistance(request->origin, i);
        for (int x = i+1; x <= j; x++) {

            if (battery[x] - extra_distance_origin <= 0 || sequence.at(x)->type==Node::Type::CHARGING_STATION) return false;
        }
        if (battery.at(j) - extra_distance_origin - inst.getTravelTime(sequence.at(j), request->destination) <= 0) return false;
        double extra_distance_destination = getAddedDistance(request->destination, j);
        double total_extra_distance = extra_distance_origin + extra_distance_destination;
        for (int x = j + 1; x < sequence.size(); x++) {
            if (battery[x] - total_extra_distance <= 0) return false;
        }
        return true;
    }
}