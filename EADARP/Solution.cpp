#include "Solution.h"
#include "Instance.h"
#include <cfloat>
#include <iostream>
#include <numeric>
#include <map>

Solution::Solution(){
    total_travel_distance = total_excess_ride_time = 0.0;
    rejected.reserve(inst.requests.size());
    removed.reserve(inst.requests.size()/2);
    for (CStation* s : inst.charging_stations) stationVisits[s] = 0;
}

double Solution::objectiveValue() const
{
    return 0.75*total_travel_distance+0.25*total_excess_ride_time+rejected.size()*(inst.avgDistance*inst.requests.size());
}

void Solution::addRoute(const Route& r)
{
    // In case we are updating the vehicle's route...
    if (routes.contains(r.vehicle)) { 
        total_travel_distance-=routes[r.vehicle].travel_distance;
        total_excess_ride_time-=routes[r.vehicle].excess_ride_time;
        for (CStation* s : inst.charging_stations) if (routes[r.vehicle].assigned_cs[s]) stationVisits[s]--;
    }
    total_travel_distance+=r.travel_distance;
    total_excess_ride_time+=r.excess_ride_time;
    for (CStation* s : inst.charging_stations) if (r.assigned_cs.at(s)) stationVisits[s]++;
    routes[r.vehicle] = r;
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
    }
    else printf("%f,%d\n",objectiveValue(),rejected.size());
}

void Solution::AddDepots()
{
    for (EAV* v : inst.vehicles)
    {
        Route r = Route(v);
        r.insertNode(inst.getDepot(v, "start"), 0);
        r.insertNode(inst.getDepot(v, "end"), 1);
        r.updateMetrics();
        addRoute(r);
    }
}

double Solution::getInsertionCost(Request* r, Position p) {
    
    double current_cost, new_cost;
    Route old_route, test_route;
    
    current_cost = objectiveValue();
    old_route = routes[p.vehicle];
    test_route = old_route;
    if (p.cs_pos.size()) {
        for (const auto& [station, node] : p.cs_pos) {
            int index = test_route.node_indices[node] + 1;
            std::for_each(test_route.path.begin() + index, test_route.path.end(), [&test_route](Node* node)
                {test_route.node_indices[node]++;}
            );
            test_route.insertNode(inst.nodes[station->id - 1],index);
            test_route.node_indices[inst.nodes[station->id - 1]] = index;
            inst.operationsSaved++;
        }
    }
    test_route.insertRequest(r, p.origin_pos + 1, p.dest_pos + 1);
    test_route.updateMetrics();
    if (test_route.isFeasible()) {
        addRoute(test_route);
        new_cost = objectiveValue();
        addRoute(old_route);
        return new_cost - current_cost;
    }
    else return DBL_MAX;
   
}


