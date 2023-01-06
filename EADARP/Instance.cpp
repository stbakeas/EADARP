#include "Instance.h"
#include <iostream> // std::cerr
#include <fstream>  // std::ifstream
#include <cmath>    // sqrt, pow
#include <regex>
#include <climits>
#include <random>
#include <numeric>
#include "RandLib.h"



Instance::~Instance()
{
    for (Node* node : nodes)
        delete node;

    for (EAV* v : vehicles)
        delete v;

    for (Request* req : requests)
        delete req;

    for (CStation* station : charging_stations) {
        delete station;
    }
}

void Instance::RandomInit(int request_num, int vehicles_num, int stations_num, int max_latitude,
   int max_longitude,double planning_horizon, int seed) {
    RandLib randlib(seed);
    Horizon = planning_horizon;
    maximumCapacity = 6;
    numberOfDepots = vehicles_num;
    dischargeRate = 0.055;
    maximumBattery = 14.85;
    maxDistance = sqrt(2 *pow(abs(max_latitude)+abs(max_longitude),2));
    returnedBatteryPercentage = 0.4;

    for (size_t i = 0; i< 3; i++) ideal[i] = 0.0;
    //Create and add requests
    for (int i = 1; i <= request_num; i++)
    {
        Node* node1 = new Node(i);
        Node* node2 = new Node(i+request_num);
        node1->type = Node::Type::ORIGIN;
        node2->type = Node::Type::DESTINATION;
       
        //Set coordinates
        node1->x = randlib.unifrnd(-max_latitude, max_latitude);
        node1->y = randlib.unifrnd(-max_longitude, max_longitude);

        node2->x = randlib.unifrnd(-max_latitude, max_latitude);
        node2->y = randlib.unifrnd(-max_longitude, max_longitude);

        node1->load = randlib.randint(1,4);
        node2->load = -node1->load;

        //Set time windows
        bool inbound = (bool) randlib.randint(0,1);
        if (inbound) {
            node1->earliest = randlib.unifrnd(360, 1020);
            node1->latest = node1->earliest + 15;
            node2->earliest = 0;
            node2->latest = planning_horizon;
        }
        else {
            node1->earliest = 0;
            node1->latest = planning_horizon;
            node2->earliest = randlib.unifrnd(420,1080);
            node2->latest = node2->earliest+15;
        }
        node1->service_duration = node1->load;
        node2->service_duration = node1->service_duration;

        node1->maximum_travel_time = DBL_MAX;
        node2->maximum_travel_time = 40;

        node1->recharge_cost = 0;
        node2->recharge_cost = 0;
        node1->recharge_rate = 0;
        node2->recharge_rate = 0;

        nodes.push_back(node1);
        nodes.push_back(node2);
        requests.push_back(new Request(node1, node2,inbound?Request::Direction::INBOUND:Request::Direction::OUTBOUND));
    }
    
    //Create and add vehicles
    for (int i = 1; i <= vehicles_num; i++)
    {
        /*We will create both a starting and ending depot for the same location,
        * since we need to represent an earliest departure time when the vehicle starts service
        and a latest arrival time when the platform returns the vehicle to its owner */
        Node* node_start = new Node(2 * request_num + i); 
        Node* node_end = new Node(2 * request_num + vehicles_num + i);
        node_start->type = Node::Type::START_DEPOT;
        node_end->type = Node::Type::END_DEPOT;
        node_start->x = randlib.unifrnd(-max_latitude, max_latitude);
        node_start->y = randlib.unifrnd(-max_longitude, max_longitude);
        node_end->x = node_start->x;
        node_end->y = node_start->y;
        node_start->load = 0;
        node_end->load = 0;
        node_start->service_duration = 0;
        node_end->service_duration = 0;

        node_start->earliest = randlib.unifrnd(100,300);
        node_start->latest = planning_horizon;
        node_end->earliest = 0.0;
        node_end->latest = randlib.unifrnd(1000, planning_horizon);
        node_start->maximum_travel_time = DBL_MAX;
        node_end->maximum_travel_time = DBL_MAX;

        node_start->recharge_rate = 0;
        node_start->recharge_cost = 0;
        node_end->recharge_rate = 0;
        node_end->recharge_cost = 0;
        nodes.push_back(node_start);
        nodes.push_back(node_end);
        vehicles.push_back(new EAV(2*request_num+i, randlib.randint(4, maximumCapacity),maximumBattery,node_start->earliest,node_end->latest,returnedBatteryPercentage));
    }

    //Create and add charging stations
    for (int i = 1; i <= stations_num; i++)
    {
        Node* node3 = new Node(2 * request_num + 2*vehicles_num + i);
        node3->type = Node::Type::CHARGING_STATION;
        node3->x = randlib.unifrnd(-max_latitude, max_latitude);
        node3->y = randlib.unifrnd(-max_longitude, max_longitude);
        node3->load = 0;
        node3->service_duration = 0;
        node3->earliest = 0;
        node3->latest = planning_horizon;
        node3->maximum_travel_time = DBL_MAX;
        node3->recharge_rate = dischargeRate;
        node3->recharge_cost = 0.08;
        nodes.push_back(node3);
        charging_stations.push_back(new CStation(2*request_num+2*vehicles_num+i,node3->recharge_rate,node3->recharge_cost));
    }

    //We do this to represent a request i by (i,n+i) instead of (i,i+1)
    std::sort(nodes.begin(), nodes.end(), [](const Node* n1, const Node* n2) {return n1->id < n2->id; });
    computeNadirPoints();
    createDistanceMatrix();
    for (Request* r : requests) { 
        r->reward = getTravelTime(r->origin, r->destination); 
        ideal[2] -= r->reward;
    }
    Preprocessing();
}

void Instance::loadFromFile(const std::string instance_file_name, int seed) {
    Horizon = 1440;
    maximumCapacity = 8;
    dischargeRate = 0.055;
    maximumBattery = 14.85;
    returnedBatteryPercentage = 0.4;
    maxDistance = sqrt(2 * pow(abs(-10) + abs(10), 2));
    for (size_t i = 0; i < 3; i++) ideal[i] = 0.0;
    RandLib randlib(seed);
    std::ifstream file(instance_file_name);

    if (!file.is_open()) {
        std::cerr << "Failed to read file '" << instance_file_name << '\'' << std::endl;
        exit(1);
    }

    // Header metadata
    int vehicles_num, requests_num;

    file >> vehicles_num;
    file >> requests_num;


    int max_route_duration;
    std::vector<int> vehicle_capacities(4);

    // Add vehicles
    for (int i = 1; i <= vehicles_num; i++) {
        file >> max_route_duration;
        for (size_t i = 0; i < 4; i++)
        {
            file >> vehicle_capacities[i];
        }
        int total_capacity = std::accumulate(vehicle_capacities.begin(), vehicle_capacities.end(), 0.0);
        int start_time = randlib.randint(0, Horizon - max_route_duration);
        int end_time = start_time + max_route_duration;

        Node* node_start = new Node(2 * requests_num + i);
        Node* node_end = new Node(2 * requests_num +vehicles_num+i);
        node_start->type = Node::Type::START_DEPOT;
        node_end->type = Node::Type::END_DEPOT;
        if (i % vehicles_num == 1) {
            node_start->x = -5;
            node_start->y = -5;
        }
        else if (i % vehicles_num == 2) {
            node_start->x = 5;
            node_start->y = 5;
        }
        else if (i % vehicles_num == 3) {
            node_start->x = -5;
            node_start->y = 5;
        }
        else {
            node_start->x = 5;
            node_start->y = -5;
        }

        node_end->x = node_start->x;
        node_end->y = node_start->y;
        node_start->load = 0;
        node_end->load = 0;
        node_start->service_duration = 0;
        node_end->service_duration = 0;

        node_start->earliest = start_time;
        node_start->latest = Horizon;
        node_end->earliest = 0.0;
        node_end->latest = end_time;
        node_start->maximum_travel_time = DBL_MAX;
        node_end->maximum_travel_time = DBL_MAX;

        node_start->recharge_rate = 0;
        node_start->recharge_cost = 0;
        node_end->recharge_rate = 0;
        node_end->recharge_cost = 0;
        nodes.push_back(node_start);
        nodes.push_back(node_end);

        vehicles.push_back(new EAV(2 * requests_num + i,total_capacity, 14.85, start_time, end_time, returnedBatteryPercentage));
    }

    // Build all request nodes
    for (int id = 0; file >> id; ) {
        if (!id) file.ignore(1000, '\n');
        else if (id == 2 * requests_num + 1) break;
        else {
            Node* node = new Node(id);
            file >> node->x;
            file >> node->y;
            file >> node->service_duration;
            file >> node->maximum_travel_time;
            std::vector<int> loads(4);
            for (int i = 0; i < loads.size(); i++) file >> loads.at(i);
            node->load = std::accumulate(loads.begin(), loads.end(), 0.0);

            file >> node->earliest;
            file >> node->latest;

            node->recharge_cost = 0;
            node->recharge_rate = 0;

            // Add type of node
            if (node->load > 0)
                node->type = Node::Type::ORIGIN;
            else if (node->load < 0)
                node->type = Node::Type::DESTINATION;


            nodes.push_back(node);
        }
    }
    file.close();

    std::sort(nodes.begin(), nodes.end(), [](const Node* n1, const Node* n2) {return n1->id < n2->id; });

    // Add all requests
    for (int i = 0; i < requests_num; i++){
        nodes.at(requests_num + i)->maximum_travel_time = nodes.at(i)->maximum_travel_time;
        nodes.at(i)->maximum_travel_time = DBL_MAX;
        requests.push_back(new Request(nodes.at(i), nodes.at(requests_num + i),
            nodes.at(i)->latest < inst.Horizon?Request::Direction::INBOUND:Request::Direction::OUTBOUND));
    }


    int stations_num = 5;
    //Create and add charging stations
    for (int i = 1; i <= stations_num; i++)
    {
        Node* node3 = new Node(2 * requests_num + 2 * vehicles_num + i);
        node3->type = Node::Type::CHARGING_STATION;
        node3->x = randlib.unifrnd(-10,10);
        node3->y = randlib.unifrnd(-10,10);
        node3->load = 0;
        node3->service_duration = 0;
        node3->earliest = 0;
        node3->latest = Horizon;
        node3->maximum_travel_time = DBL_MAX;
        node3->recharge_rate = dischargeRate;
        node3->recharge_cost = 0.08;
        nodes.push_back(node3);
        charging_stations.push_back(new CStation(2 * requests_num + 2 * vehicles_num + i, node3->recharge_rate, node3->recharge_cost));
    }

    computeNadirPoints();
    createDistanceMatrix();
    for (Request* r : requests) {
        r->reward = getTravelTime(r->origin, r->destination);
        ideal[2] -= r->reward;
    }
    Preprocessing();
}

void Instance::createDistanceMatrix()
{
    distanceMatrix.resize(nodes.size());
    for (int i = 0; i < distanceMatrix.size(); i++) {
        distanceMatrix[i].resize(nodes.size());

        for (int j = 0; j < distanceMatrix.size(); j++) {
            distanceMatrix[i][j] = sqrt(
                pow(nodes[i]->x - nodes[j]->x, 2) + pow(nodes[i]->y - nodes[j]->y, 2)
            );
        }
    }
}

void Instance::computeNadirPoints() {

    nadir[0] = Horizon;
    
    //Owner Inconvenience
    nadir[1] = inst.vehicles.size() * maximumBattery * returnedBatteryPercentage;
    for (EAV* v : inst.vehicles) nadir[1] += Horizon - v->end_time;
    
    //System Cost
    nadir[2] = 0.0;
    for (CStation* s : inst.charging_stations) nadir[2] += inst.vehicles.size()*s->getRequiredTime(0.0, maximumBattery) * s->recharge_cost;
}

void Instance::Preprocessing() {
   
    int n = inst.requests.size();
    for (int i = 0; i < distanceMatrix.size(); i++) {
        for (int j = 0; j < distanceMatrix.size(); j++) {
            if (i != j) {
                if ((abs(nodes[i]->load + nodes[j]->load) > inst.maximumCapacity) ||
                    (nodes[i]->isStartingDepot() && nodes[j]->isDestination()) ||
                    (nodes[i]->isOrigin() && nodes[j]->isEndingDepot()) ||
                    (nodes[i]->isOrigin() && nodes[j]->isChargingStation()) ||
                    (nodes[i]->isChargingStation() && nodes[j]->isDestination()) ||
                    (nodes[i]->isDestination() && nodes[j]->isOrigin() && i == j + n)
                    )
                    distanceMatrix[i][j] = DBL_MAX;
                    
            } 
        }
    }
    int request_count;
    for (EAV* v : inst.vehicles) {
        int count[2] = { 0,0 };
        Node* start_depot = inst.getDepot(v, "start");
        Node* end_depot = inst.getDepot(v, "end");
        std::cout << v->end_time << std::endl;
        for (Request* r: inst.requests) {
            if (r->direction == Request::Direction::INBOUND) {
                if (v->start_time + inst.getTravelTime(start_depot, r->origin) > r->origin->latest) {
                    r->forbidden_vehicles[static_cast<int>(Objective::User)].insert(v->id);
                    count[static_cast<int>(Objective::User)]++;
                }
                   
                if (r->origin->earliest + r->origin->service_duration + inst.getTravelTime(r->origin, r->destination) +
                    r->destination->service_duration + inst.getTravelTime(r->destination, end_depot) > v->end_time) {
                    r->forbidden_vehicles[static_cast<int>(Objective::Owner)].insert(v->id);
                    count[static_cast<int>(Objective::Owner)]++;
                }
                   


            }
            else {
                if (v->start_time + inst.getTravelTime(start_depot, r->origin) + r->origin->service_duration +
                    inst.getTravelTime(r->origin, r->destination) > r->destination->latest) {
                    r->forbidden_vehicles[static_cast<int>(Objective::User)].insert(v->id);
                    count[static_cast<int>(Objective::User)]++;
                }
                if (r->destination->earliest + r->destination->service_duration +
                    inst.getTravelTime(r->destination, end_depot) > v->end_time) {
                    r->forbidden_vehicles[static_cast<int>(Objective::Owner)].insert(v->id);
                    count[static_cast<int>(Objective::Owner)]++;
                }
            }
        }
        std::cout << "Vehicle " << v->id << " has " << count[0] << " forbidden requests for user & " << count[1] << " for owner.\n";
    }
    for (int i = 1; i < 9; i++) {
        for (int j = 1; j < 9 - i + 1; j++) {
            std::vector<float> coefficients = { static_cast<float>(i) / 10,static_cast<float>(j) / 10,static_cast<float>(10 - i - j) / 10 };
            weight_combinations.push_back(coefficients);
        }
    }

}

Instance& Instance::getUnique()
{
    static Instance unique;
    return unique;
}

Request* Instance::getRequest(Node* node)
{
    return node->isOrigin()? requests[node->id - 1] : requests[node->id - requests.size() - 1];
}

CStation* Instance::getChargingStation(Node* node) {
    return charging_stations.at(node->id-2*requests.size()-2*vehicles.size()-1);
}

Node* Instance::getDepot(EAV* vehicle,std::string start_or_end)
{
    if (start_or_end == "start") return nodes.at(vehicle->id-1);
    else return nodes.at(vehicles.size()+vehicle->id-1);
}

double Instance::getTravelTime(Node* n1, Node* n2)
{
    return distanceMatrix[n1->id-1][n2->id-1];
}

double Instance::getFuelConsumption(Node* n1, Node* n2)
{
    return dischargeRate * inst.getTravelTime(n1, n2);
}

bool Instance::isForbiddenArc(Node* n1, Node* n2) {
    return inst.getTravelTime(n1, n2) == DBL_MAX;
}