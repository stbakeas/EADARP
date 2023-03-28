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
}

double Solution::objectiveValue() const
{
    return total_travel_distance;
}

void Solution::addRoute(Route r)
{
    // In case we are updating the vehicle's route...
    if (routes.contains(r.vehicle)) { 
        total_travel_distance-=routes[r.vehicle].travel_distance;
        total_excess_ride_time-=routes[r.vehicle].excess_ride_time;
    }
    total_travel_distance+=r.travel_distance;
    total_excess_ride_time+=r.excess_ride_time;
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
                    my_route.second.path.at(i)->id, ",", my_route.second.battery.at(i), "kWh,", my_route.second.start_of_service_times.at(i), ")->");
            }
            printf("%s%i%s%f%s%f%s",
            "(" , my_route.second.path.back()->id , "," , my_route.second.battery.back() , "kWh," , my_route.second.start_of_service_times.back() , ")");
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

void Solution::FixHardConstraints() {
    for (EAV* v:inst.vehicles) {
        std::vector<Request*> removalPool;
        removalPool.reserve(routes[v].path.size() - 2);
        while (!routes[v].capacityFeasible) {
            for (size_t i = 0; i < routes[v].path.size(); i++)
            {
                if (routes[v].loads[i] > v->capacity) {
                    Request* request = inst.getRequest(routes[v].path[i]);
                    Route route = routes[v];
                    removalPool.push_back(request);
                    route.removeRequest(request);
                    route.updateMetrics();
                    addRoute(route);
                    break;
                }
            }
        }
        for (Request* r : removalPool) {
            std::vector<Position> capset;
            for (size_t i = 0; i < routes[v].path.size(); ++i)
            {
                if (inst.isForbiddenArc(routes[v].path.at(i), r->origin)) continue;
                for (size_t j = i; j < routes[v].path.size(); ++j)
                {
                    if (inst.isForbiddenArc(routes[v].path.at(j), r->destination)) continue;
                    if (routes[v].isInsertionCapacityFeasible(r, i, j)) capset.emplace_back(v, i, j, nullptr, -1);

                }
            }
            if (capset.empty()) rejected.push_back(r);
            else {
                std::sort(capset.begin(), capset.end(), [this,r](Position p1,Position p2) {
                    return routes[p1.vehicle].getAddedDistance(r, p1.origin_pos, p1.dest_pos) <
                        routes[p2.vehicle].getAddedDistance(r, p2.origin_pos, p2.dest_pos);

                });
                Route route = routes[v];
                route.insertRequest(r, capset[0].origin_pos + 1, capset[0].dest_pos + 1);
                route.updateMetrics();
                addRoute(route);
            }
        }
        if (!routes[v].batteryFeasible) {
            std::vector<int> zero_load_nodes;
            zero_load_nodes.reserve(routes[v].path.size() / 2);
            for (size_t i = 0; i < routes[v].battery.size()-1; i++)
            {
                if (!(routes[v].battery[i] < 0.0) && !routes[v].loads[i]) zero_load_nodes.push_back(i);
                if (routes[v].battery[i] < 0.0) break;
            }
            for (size_t i=zero_load_nodes.size()-1;i>0;i--) {
                CStation* cs = routes[v].findBestChargingStationAfter(zero_load_nodes[i]);
                if (cs != nullptr) {
                    Route route = routes[v];
                    route.insertNode(inst.nodes.at(cs->id - 1), zero_load_nodes[i] + 1);
                    route.updateMetrics();
                    addRoute(route);
                    if (route.batteryFeasible) break;
                }
            }
        }
        while (!routes[v].batteryFeasible) {
            RandLib randlib(static_cast<unsigned int>(std::time(nullptr)));
            Request* r = routes[v].selectRandomRequest(randlib);
            rejected.push_back(r);
            Route route = routes[v];
            route.removeRequest(r);
            route.updateMetrics();
            addRoute(route);
        }
    }
}

double Solution::getInsertionCost(Request* r, Position p) {
    
    double current_cost, new_cost;
    Route old_route, test_route;
    
    current_cost = objectiveValue();
    old_route = routes[p.vehicle];
    test_route = old_route;
    if (p.charging_station != nullptr) test_route.insertNode(inst.nodes[p.charging_station->id - 1], p.cs_pos + 1);
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


