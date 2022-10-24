#include "Instance.h"
#include <iostream> // std::cerr
#include <fstream>  // std::ifstream
#include <cmath>    // sqrt, pow
#include <regex>
#include <climits>
#include <random>
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

        node1->load = randlib.randint(1, 4);
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

        node1->maximum_travel_time = 30;
        node2->maximum_travel_time = 30;

        node1->recharge_cost = 0;
        node2->recharge_cost = 0;
        node1->recharge_rate = 0;
        node2->recharge_rate = 0;

        nodes.push_back(node1);
        nodes.push_back(node2);
        requests.push_back(new Request(node1, node2));
    }
    
    //Create and add vehicles
    for (int i = 1; i <= vehicles_num; i++)
    {
        /*We will create both a starting and ending depot for the same location,
        * since we need to represent an earliest departure time when the vehicle starts service
        and a latest arrival time when the platform returns the vehicle to its owner */
        Node* node_start = new Node(2 * request_num + i); 
        Node* node_end = new Node(2 * request_num + vehicles_num + i);
        node_start->type = Node::Type::DEPOT;
        node_end->type = Node::Type::DEPOT;
        node_start->x = randlib.unifrnd(-max_latitude, max_latitude);
        node_start->y = randlib.unifrnd(-max_longitude, max_longitude);
        node_end->x = node_start->x;
        node_end->y = node_start->y;
        node_start->load = 0;
        node_end->load = 0;
        node_start->service_duration = 0;
        node_end->service_duration = 0;

        node_start->earliest = randlib.unifrnd(0.0,400.0);
        node_start->latest = planning_horizon;
        node_end->earliest = 0.0;
        node_end->latest = randlib.unifrnd(1000.0, planning_horizon);
        node_start->maximum_travel_time = DBL_MAX;
        node_end->maximum_travel_time = DBL_MAX;

        node_start->recharge_rate = 0;
        node_start->recharge_cost = 0;
        node_end->recharge_rate = 0;
        node_end->recharge_cost = 0;
        nodes.push_back(node_start);
        nodes.push_back(node_end);
        vehicles.push_back(new EAV(2*request_num+i, randlib.randint(4, 5),400,node_start->earliest,node_end->latest,randlib.unifrnd(0.1, 0.4)));
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
        node3->recharge_cost = randlib.randint(4,10);
        node3->recharge_rate = 0.25 * node3->recharge_cost;
        nodes.push_back(node3);
        charging_stations.push_back(new CStation(2*request_num+2*vehicles_num+i,node3->recharge_rate,node3->recharge_cost));
    }

    //We do this to represent a request i by (i,n+i) instead of (i,i+1)
    std::sort(nodes.begin(), nodes.end(), [](const Node* n1, const Node* n2) {return n1->id < n2->id; });
    createDistanceMatrix();
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
    if (start_or_end == "start") return nodes.at(vehicle->id - 1);
    else return nodes.at(vehicle->id + inst.vehicles.size() - 1);
}

double Instance::getTravelTime(Node* n1, Node* n2)
{
    return distanceMatrix[n1->id-1][n2->id-1];
}