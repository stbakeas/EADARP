#include "Instance.h"
#include <iostream> // std::cerr
#include <fstream>  // std::ifstream
#include <cmath>    // sqrt, pow
#include <regex>
#include <climits>
#include <random>
#include <numeric>
#include "RandLib.h"
#include <set>



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



void Instance::loadMalheiros(const std::string instance_file_name, int seed) {
    Horizon = 1440;
    dischargeRate = 0.055;
    maximumBattery = 14.85;
    returnedBatteryPercentage = 0.7;
    maxDistance = sqrt(2 * pow(abs(-10) + abs(10), 2));
    RandLib randlib(seed);
    std::ifstream file(instance_file_name);

    if (!file.is_open()) {
        std::cerr << "Failed to read file '" << instance_file_name << '\'' << std::endl;
        exit(1);
    }

    // Header metadata
    int vehicles_num, requests_num;

    file >> vehicles_num;
    vehicles.reserve(vehicles_num);

    file >> requests_num;
    requests.reserve(requests_num);

    int stations_num=3;
    /*printf("Enter number of charging stations : ");
    std::cin >> stations_num;*/

    nodes.reserve(2 * requests_num + 2 * vehicles_num + stations_num);


    int max_route_duration;
    std::vector<int> vehicle_capacities(4);

    // Add vehicles
    for (int i = 1; i <= vehicles_num; i++) {
        file >> max_route_duration;
        for (size_t i = 0; i < 4; i++)
        {
            file >> vehicle_capacities[i];
        }
        int total_capacity = ceil(std::accumulate(vehicle_capacities.begin(), vehicle_capacities.end(), 0)/2);
        if (total_capacity > maximumCapacity) maximumCapacity = total_capacity;
        double start_time; //randlib.randint(0, Horizon - max_route_duration);
        double end_time(Horizon); // start_time + max_route_duration;

        Node* node_start = new Node(2 * requests_num + i);
        Node* node_end = new Node(2 * requests_num +vehicles_num+i);
        node_start->type = Node::Type::START_DEPOT;
        node_end->type = Node::Type::END_DEPOT;
        if (i % vehicles_num == 1) {
            node_start->x = -5.0;
            node_start->y = -5.0;
        }
        else if (i % vehicles_num == 2) {
            node_start->x = 5.0;
            node_start->y = 5.0;
        }
        else if (i % vehicles_num == 3) {
            node_start->x = -5.0;
            node_start->y = 5.0;
        }
        else {
            node_start->x = 5.0;
            node_start->y = -5.0;
        }

        node_end->x = node_start->x;
        node_end->y = node_start->y;
        node_start->load = 0;
        node_end->load = 0;
        node_start->service_duration = double();
        node_end->service_duration = 0.0;

        node_start->earliest = start_time;
        node_start->latest = Horizon;
        node_end->earliest = 0.0;
        node_end->latest = end_time;
        node_start->maximum_travel_time = DBL_MAX;
        node_end->maximum_travel_time = DBL_MAX;

        nodes.push_back(node_start);
        nodes.push_back(node_end);

        vehicles.push_back(new EAV(2 * requests_num + i,total_capacity, maximumBattery, start_time, end_time,maximumBattery,returnedBatteryPercentage));
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
            for (int i = 0; i < loads.size(); i++) file >> loads[i];
            node->load = std::accumulate(loads.begin(), loads.end(), 0.0);

            file >> node->earliest;
            file >> node->latest;

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
        nodes[requests_num + i]->maximum_travel_time = nodes[i]->maximum_travel_time;
        nodes[i]->maximum_travel_time = DBL_MAX;

        requests.push_back(new Request(nodes[i], nodes[requests_num + i]));
    }

    //Create and add charging stations
    for (int i = 1; i <= stations_num; i++)
    {
        Node* node3 = new Node(2 * requests_num + 2 * vehicles_num + i);
        node3->type = Node::Type::CHARGING_STATION;
        node3->x = randlib.unifrnd(-10,10);
        node3->y = randlib.unifrnd(-10, 10);
        node3->load = 0;
        node3->service_duration = 0.0;
        node3->earliest = 0.0;
        node3->latest = Horizon;
        node3->maximum_travel_time = DBL_MAX;
        nodes.push_back(node3);
        charging_stations.push_back(new CStation(2 * requests_num + 2 * vehicles_num + i, 0.055));
    }

    createDistanceMatrix();
    createSimilarityMatrix();
    Preprocessing();
}

void Instance::loadCordeau(const std::string instance_file_name) {
   
    std::ifstream file(instance_file_name);

    if (!file.is_open()) {
        std::cerr << "Failed to read file '" << instance_file_name << '\'' << std::endl;
        exit(1);
    }

    // Header metadata
    int vehicles_num, requests_num, start_depots_num, end_depots_num, stations_num, station_copies;

    file >> vehicles_num;
    vehicles.reserve(vehicles_num);

    file >> requests_num;
    requests.reserve(requests_num);

    file >> start_depots_num >> end_depots_num >> stations_num >> station_copies;

    charging_stations.reserve(station_copies * stations_num);

    file >> Horizon;

    int id;
    // Build all nodes
    for (int i = 0; i<2*requests_num+2*(vehicles_num+1)+stations_num;i++) {
        file >> id;
        Node* node = new Node(id);
        file >> node->x;
        file >> node->y;
        file >> node->service_duration;
        file >> node->load;
        file >> node->earliest;
        file >> node->latest;
        node->maximum_travel_time = 120.0;

        // Add type of node
        if (node->load > 0)
            node->type = Node::Type::ORIGIN;
        else if (node->load < 0)
            node->type = Node::Type::DESTINATION;

        nodes.push_back(node);
    }

    int common_origin_id, common_dest_id;
    file >> common_origin_id;
    nodes[common_origin_id - 1]->type = Node::Type::START_DEPOT;

    file >> common_dest_id;
    nodes[common_dest_id - 1]->type = Node::Type::END_DEPOT;

    for (int i = 0; i < vehicles_num; i++) {
        file >> id;
        nodes[id-1]->type = Node::Type::START_DEPOT;
    }

    for (int i = 0; i < vehicles_num; i++) {
        file >> id;
        nodes[id-1]->type = Node::Type::END_DEPOT;
    }

    for (int i = 0; i < stations_num; i++) {
        file >> id;
        charging_stations.push_back(new CStation());
        charging_stations[i]->id = id;
        nodes[id-1]->type = Node::Type::CHARGING_STATION;
    }
    
    double ride_time;
    for (int i = 0; i < requests_num; i++) {
        file >> ride_time;
        nodes[i + requests_num]->maximum_travel_time = ride_time;
        requests.push_back(new Request(nodes[i], nodes[i + requests_num]));
    }

    int capacity;
    for (int i = 0; i < vehicles_num; i++) {
        
        file >> capacity;
        if (capacity > maximumCapacity) maximumCapacity = capacity;
        vehicles.push_back(new EAV());
        vehicles[i]->capacity = capacity;
    }

    //Skip initial battery level
    double initial_battery;
    for (int i = 0; i < vehicles_num; i++) { 
        file >> initial_battery;
        vehicles[i]->initial_battery = initial_battery;
    }
        
   double maximum_battery;
    for (int i = 0; i < vehicles_num; i++) {
        file >> maximum_battery;
        if (maximumBattery < maximum_battery) maximumBattery = maximum_battery;
        vehicles[i]->id = 2*requests_num+i+3;
        vehicles[i]->start_time = 0.0;
        vehicles[i]->end_time = Horizon;
        vehicles[i]->total_battery = maximum_battery;
        vehicles[i]->acquisition_cost = 0.0;
    }

    double return_percentage;
    for (int i = 0; i < vehicles_num; i++) {
        file >> return_percentage;
        vehicles[i]->battery_return_percentage = return_percentage;
    }

    double recharge_rate;
    for (int i = 0; i < stations_num; i++) {
        file >> recharge_rate;
        charging_stations[i]->recharge_rate = recharge_rate;
    }
    file >> dischargeRate;

    file.close();

    createDistanceMatrix();
    createSimilarityMatrix();
    Preprocessing();
}

void Instance::createDistanceMatrix()
{
    distanceMatrix.resize(nodes.size());
    for (int i = 0; i < distanceMatrix.size(); i++) {
        distanceMatrix[i].resize(nodes.size());

        for (int j = 0; j < distanceMatrix.size(); j++) {
            distanceMatrix[i][j] =sqrt(
                pow(nodes[i]->x - nodes[j]->x, 2) + pow(nodes[i]->y - nodes[j]->y, 2)
            );
        }
    }
}

void Instance::createSimilarityMatrix()
{
    //Request Similarity
    std::pair<int, int> loadDifference = { INT_MAX,0 };
    std::pair<double, double> timeWindow = { DBL_MAX,0.0 };
    std::pair<double, double> distances = { DBL_MAX,0.0 };

    for (Request* r1 : requests) {
        for (Request* r2 : requests) {
            if (r1->origin->id != r2->origin->id) {
                int loadDiff = abs(r1->origin->load - r2->origin->load);
                if (loadDiff < loadDifference.first) loadDifference.first = loadDiff;
                if (loadDiff > loadDifference.second) loadDifference.second = loadDiff;

                double timeWindowRelatedness = abs(r1->origin->earliest - r2->origin->earliest) + abs(r1->origin->latest - r2->origin->latest) +
                    abs(r1->destination->latest - r2->destination->latest) + abs(r1->destination->earliest - r2->destination->earliest);
                if (timeWindowRelatedness < timeWindow.first) timeWindow.first = timeWindowRelatedness;
                if (timeWindowRelatedness > timeWindow.second) timeWindow.second = timeWindowRelatedness;

                double dist = inst.getTravelTime(r1->origin, r2->origin) + inst.getTravelTime(r1->destination, r2->destination);
                if (dist < distances.first) distances.first = dist;
                if (dist > distances.second) distances.second = dist;
            }
        }
    }
    similarity.resize(requests.size());
    for (Request* r1 : requests) {
        similarity[r1->origin->id - 1].resize(requests.size());
        for (Request* r2 : requests) {
            int normloadDiff = (abs(r1->origin->load - r2->origin->load) - loadDifference.first);
            if (loadDifference.second - loadDifference.first) normloadDiff /= loadDifference.second - loadDifference.first;


            double normtimeWindowRelatedness = (abs(r1->origin->earliest - r2->origin->earliest) + abs(r1->origin->latest - r2->origin->latest) +
                abs(r1->destination->latest - r2->destination->latest) + abs(r1->destination->earliest - r2->destination->earliest) - timeWindow.first) /
                (timeWindow.second - timeWindow.first);

            double normDist = (inst.getTravelTime(r1->origin, r2->origin) + inst.getTravelTime(r1->destination, r2->destination) - distances.first) /
                (distances.second - distances.first);
            similarity[r1->origin->id - 1][r2->origin->id - 1] = normloadDiff + normtimeWindowRelatedness + normDist;

        }
    }
}

void Instance::Preprocessing() {

    //Time Window Tightening
    for (int i = 0; i < requests.size(); i++) {
        if (nodes[i]->latest < inst.Horizon) {

            nodes[i + requests.size()]->earliest = std::max(0.0, nodes[i]->earliest + nodes[i]->service_duration +
                getTravelTime(nodes[i], nodes[i + requests.size()]));
            nodes[i + requests.size()]->latest = std::min(Horizon, nodes[i]->latest + nodes[i]->service_duration + nodes[i + requests.size()]->maximum_travel_time);

        }
        else {
            nodes[i]->earliest = std::max(0.0, nodes[i + requests.size()]->earliest - nodes[i]->service_duration - nodes[i + requests.size()]->maximum_travel_time);
            nodes[i]->latest = std::min(Horizon, nodes[i + requests.size()]->latest - nodes[i]->service_duration - inst.getTravelTime(nodes[i], nodes[i + requests.size()]));
        }
    }
   
    //Arc Elimination
    int n = inst.requests.size();
    for (int i = 0; i < distanceMatrix.size(); i++) {
        for (int j = 0; j < distanceMatrix.size(); j++) {
            if (i != j) {
                if (nodes[i]->isOrigin() && getTravelTime(nodes[i], nodes[j]) + nodes[j]->service_duration + getTravelTime(nodes[j], nodes[i + n]) > nodes[i + n]->maximum_travel_time) {
                    distanceMatrix[i][j] = DBL_MAX;
                    distanceMatrix[j][n + i] = DBL_MAX;
                }

                if ((abs(nodes[i]->load + nodes[j]->load) > inst.maximumCapacity) ||
                    (nodes[i]->isStartingDepot() && nodes[j]->isDestination()) ||
                    (nodes[i]->isOrigin() && nodes[j]->isEndingDepot()) ||
                    (nodes[i]->isOrigin() && nodes[j]->isChargingStation()) ||
                    (nodes[i]->isChargingStation() && nodes[j]->isDestination()) ||
                    (nodes[i]->isDestination() && nodes[j]->isOrigin() && i == j + n) ||
                    (nodes[i]->earliest + nodes[i]->service_duration + getTravelTime(nodes[i], nodes[j]) > nodes[j]->latest)
                    /*||
                    (nodes[i]->isEndingDepot()) ||
                    (nodes[j]->isStartingDepot())*/
                    )
                {
                    distanceMatrix[i][j] = DBL_MAX;
                }

            } 

        }
    }
    
    //Incompatible request-vehicle pairs
    for (Request* r : inst.requests){
        for (EAV* v: inst.vehicles) {
            Node* start_depot = inst.getDepot(v, "start");
            Node* end_depot = inst.getDepot(v, "end");
            bool violation_origin = v->start_time + inst.getTravelTime(start_depot, r->origin) > r->origin->latest;
            bool violation_dest = v->start_time + inst.getTravelTime(start_depot, r->origin) + r->origin->service_duration +
                inst.getTravelTime(r->origin, r->destination) > r->destination->latest;
            bool violation_vehicle = std::max(r->destination->earliest, r->origin->earliest + r->origin->service_duration + inst.getTravelTime(r->origin, r->destination))
                + r->destination->service_duration + inst.getTravelTime(r->destination, end_depot) > v->end_time;
            if (violation_origin || violation_dest || violation_vehicle) r->forbidden_vehicles.insert(v->id);
        }
        printf("%s%i%s%i%s\n", "Request " , r->origin->id , " has " , r->forbidden_vehicles.size() , " forbidden vehicles");
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
    return charging_stations[node->id-2*requests.size()-2*(vehicles.size()+1)-1];
}

Node* Instance::getDepot(EAV* vehicle,std::string start_or_end)
{
    if (start_or_end == "start") return nodes[vehicle->id-1];
    else return nodes[vehicles.size()+vehicle->id-1];
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