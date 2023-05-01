#include "Solution.h"
#include "Instance.h"
#include <cfloat>
#include <iostream>
#include <numeric>
#include <map>

Solution::Solution() :total_travel_distance(0.0),total_excess_ride_time(0.0){
    for (CStation* s : inst.charging_stations) stationVisits[s] = 0;
    rejected.reserve(inst.requests.size());
    removed.reserve(inst.requests.size());

}

double Solution::objectiveValue() const noexcept
{
    return 0.75*total_travel_distance+0.25*total_excess_ride_time+inst.rejectedRequestPenalties[rejected.size()];
}

void Solution::addRoute(const Route& r)
{
    // In case we are updating the vehicle's route...
    if (routes.contains(r.vehicle)) { 
        total_travel_distance-=routes[r.vehicle].travel_distance;
        total_excess_ride_time-=routes[r.vehicle].excess_ride_time;
        for (const auto& [station,amount] : routes[r.vehicle].desiredAmount) stationVisits[station]--;
    }
    total_travel_distance+=r.travel_distance;
    total_excess_ride_time+=r.excess_ride_time;
    for (const auto& [station, amount] : r.desiredAmount) stationVisits[station]++;
    routes[r.vehicle] = std::move(r);
}

void Solution::deleteEmptyRoutes()
{
    for (auto it = routes.begin(); it != routes.end(); )
        if (it->second.hasNoRequests())
            it = routes.erase(it);
        else
            ++it;
}

void Solution::Display(int i) {
    if (i == 0) {
        for (auto my_route : routes) {
            printf ("%s%i%s%i%s","Route ID: ",my_route.first->id," with size ",my_route.second.path.size()," : ");
            size_t length = my_route.second.path.size();
            for (size_t i = 0; i < length - 1; i++)
            {
                printf("%s%i%s%f%s%f%s","(",
                    my_route.second.path.at(i)->id, ",", dbl_round(my_route.second.battery.at(i),3), "kWh,", dbl_round(my_route.second.start_of_service_times.at(i),2), ")->");
            }
            printf("%s%i%s%f%s%f%s",
            "(" , my_route.second.path.back()->id , "," , dbl_round(my_route.second.battery.back(), 3), "kWh," , dbl_round(my_route.second.start_of_service_times.back(), 2), ")");
            printf("\n\n");
        }
        for (Request* request : rejected) {
            std::cout << request->origin->id << "\n";
        }
    }
    else printf("%f,%d\n",objectiveValue(),rejected.size());
}

void Solution::AddDepots()
{
    for (EAV* v : inst.vehicles)
    {
        Route r = Route(v);
        r.path.resize(2);
        r.path[0]=inst.getDepot(v,1);
        r.path[1] =inst.getDepot(v,0);
        r.travel_distance += inst.getTravelTime(r.path[0], r.path[1]);
        r.node_indices[r.path[0]] = 0;
        r.node_indices[r.path[1]] = 1;
        r.updateMetrics();
        addRoute(r);
    }
}

double Solution::getInsertionCost(Request* r, const Position& p) {
    
    Route test_route = routes[p.vehicle];
    for (const auto& [station, node] : p.cs_pos) test_route.insertNode(inst.nodes[station->id - 1], test_route.node_indices[node] + 1);
    test_route.insertRequest(r, p.origin_pos + 1, p.dest_pos + 1);
    test_route.updateMetrics();
    return !test_route.isFeasible() ? DBL_MAX : 0.75 * (test_route.travel_distance - routes[p.vehicle].travel_distance)
        + 0.25*(test_route.excess_ride_time - routes[p.vehicle].excess_ride_time);
}


