#include "Solution.h"
#include "Instance.h"
#include <cfloat>

Solution::Solution():total_cost(0.0)
{
    
}

void Solution::addRoute(Route r)
{
    // In case we are updating the vehicle's route...
    if (routes.find(r.vehicle) != routes.end())
        total_cost -= routes[r.vehicle].cost;

    routes[r.vehicle] = r;
    total_cost += r.cost;
}

bool Solution::feasible()
{
    return routes.size() <= inst.vehicles.size();
}

void Solution::deleteEmptyRoutes()
{
    for (auto it = routes.begin(); it != routes.end(); )
        if (it->second.isEmpty())
            it = routes.erase(it);
        else
            ++it;
}