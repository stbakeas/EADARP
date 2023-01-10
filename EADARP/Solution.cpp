#include "Solution.h"
#include "Instance.h"
#include <cfloat>
#include <iostream>
#include <numeric>
#include <map>

Solution::Solution(){
    for (size_t i = 0; i < 3; i++) objective_value[i] = 0.0;
}

float float_one_point_round(float value)
{
    return ((float)((int)(value * 10))) / 10;
}

void Solution::SetWeights(std::vector<float> values) {
    weights[0] = values[0];
    weights[1] = values[1];
    weights[2] = values[2];
}

void Solution::addRoute(Route r)
{
    // In case we are updating the vehicle's route...
    if (routes.find(r.vehicle) != routes.end())
    {
        objective_value[0] -= routes[r.vehicle].user_inconvenience;
        objective_value[1] -= routes[r.vehicle].owner_inconvenience;
        objective_value[2] = objective_value[2]-(routes[r.vehicle].charging_cost-routes[r.vehicle].earnings);
    }
    objective_value[0] += r.user_inconvenience;
    objective_value[1] += r.owner_inconvenience;
    objective_value[2] += r.charging_cost - r.earnings;
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
            std::cout << "Route ID: " << my_route.first->id << " with size " << my_route.second.path.size() << " : ";
            size_t length = my_route.second.path.size();
            for (size_t i = 0; i < length - 1; i++)
            {
                std::cout << "(" << my_route.second.path.at(i)->id << "," << my_route.second.battery.at(i) << "kWh" << "," << my_route.second.loads[i] << ")->";
            }
            std::cout << "(" << my_route.second.path.back()->id << "," << my_route.second.battery.back() << "kWh" << "," << my_route.second.loads.back() << ")";
            std::cout << "\n\n";
        }
    }
    else if (i == 1) {
        std::cout << AchievementFunction(0.3) << std::endl;
    }
    else {
        std::cout << "(";
        for (int i = 0; i < 2; i++) std::cout << objective_value[i] << ",";
        std::cout << objective_value[2] << ")";
    }
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
                    Position p = Position(v, i, j, nullptr, -1);
                    if (routes[v].isInsertionCapacityFeasible(r, i, j)) capset.push_back(p);

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
            for (size_t i = 0; i < routes[v].battery.size()-1; i++)
            {
                if (routes[v].battery[i] >= 0 && !routes[v].loads[i]) zero_load_nodes.push_back(i);
                if (routes[v].battery[i] < 0) break;
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

double Solution::AchievementFunction(double rho) {
    std::vector<double> deviations;
    double maxValue;
    double first_scale;
    for (size_t i = 0; i < 3; i++) { 
        deviations.push_back(objective_value[i] - inst.ideal[i]);
        first_scale = weights[i] * deviations[i] / (inst.nadir[i] - inst.ideal[i]);
        if (i == 0) maxValue = first_scale;
        else if (first_scale > maxValue) maxValue = first_scale;
    }
    return maxValue+rho * std::accumulate(deviations.begin(), deviations.end(), 0.0);
}

double Solution::distanceFrom(Solution s, bool objectiveSpace) {
    double distance = 0;
    if (objectiveSpace) {
        for (int i = 0; i < 3; i++) distance += abs(objective_value[i] - s.objective_value[i]);
    }
    else 
    {
        for (EAV* v : inst.vehicles) {
            std::vector<Request*> all_requests = s.routes[v].findAllRequests();
            std::vector<Request*> this_requests = routes[v].findAllRequests();
            all_requests.insert(all_requests.end(), this_requests.begin(), this_requests.end());
            std::map<int, int> frequency;
            for (const auto& req : all_requests) frequency[req->origin->id]++;
            for (const auto& pair : frequency) {
                if (pair.second > 1) {
                    Request* r = inst.getRequest(inst.nodes.at(pair.first - 1));
                    all_requests.erase(std::remove(all_requests.begin(), all_requests.end(), r), all_requests.end());
                }
            }
            distance += all_requests.size();

        }

    }
    return distance;
}

double Solution::getInsertionCost(Request* r, Position p, Instance::Objective objective) {
    
    double current_cost,new_cost,cost;
    Route old_route, test_route;
    switch (objective) {
    case Instance::Objective::User: {
        current_cost = objective_value[static_cast<int>(objective)];
        old_route = routes[p.vehicle];
        test_route = old_route;
        if (p.charging_station != nullptr) {
            test_route.insertNode(inst.nodes[p.charging_station->id - 1], p.cs_pos + 1);
        }
        test_route.insertRequest(r, p.origin_pos + 1, p.dest_pos + 1);
        test_route.updateMetrics();
        addRoute(test_route);
        new_cost = objective_value[static_cast<int>(objective)];
        cost = new_cost - current_cost;
        addRoute(old_route);
        break;
    }
    case Instance::Objective::Owner: {
        break;
    }
    default: {
        current_cost = AchievementFunction(0.3);
        old_route = routes[p.vehicle];
        test_route = old_route;
        if (p.charging_station != nullptr) {
            test_route.insertNode(inst.nodes[p.charging_station->id - 1], p.cs_pos + 1);
        }
        test_route.insertRequest(r, p.origin_pos + 1, p.dest_pos + 1);
        test_route.updateMetrics();
        addRoute(test_route);
        new_cost = AchievementFunction(0.3);
        cost = new_cost - current_cost;
        addRoute(old_route);
        break;
    }
    }

    return cost;
}

bool Solution::dominates(Solution s) {
    for (int i = 0; i < 3; i++) {
        if (this->objective_value[i] > s.objective_value[i]) return false;
    }
    return true;
}

bool Solution::operator == (const Solution& s) const {
    for (int i = 0; i < 3; i++) {
        if (this->objective_value[i] != s.objective_value[i]) return false;
    }
    return true;
}

bool Solution::operator != (const Solution& s) const {
    for (int i = 0; i < 3; i++) {
        if (this->objective_value[i] != s.objective_value[i]) return true;
    }
    return false;
}
