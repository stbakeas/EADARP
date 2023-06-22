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
#include "Route.h"

Instance::~Instance()
{

    for (Node* node : nodes)
        delete node;

    for (EAV* v : vehicles)
        delete v;

    for (Request* req : requests)
        delete req;

    for (CStation* station : charging_stations) 
       delete station;
    
}
void Instance::RandomGenerator(int request_num, int vehicles_num, int stations_num, double max_latitude,
    double max_longitude, double planning_horizon, int seed, double returnBatteryPercentage) {
    RandLib randlib(seed);
    Horizon = planning_horizon;
    maximumCapacity = 4;
    dischargeRate = 0.055;
    maximumBattery = 14.85;
    this->returnedBatteryPercentage = returnBatteryPercentage;
    double maxDistance = sqrt(2 * pow(abs(max_latitude) + abs(max_longitude), 2));

    int verified = 0;
    //Create and add requests
    for (int i = 1; i <= request_num; i++)
    {
        Node* node1 = new Node(i);
        Node* node2 = new Node(i + request_num);
        node1->type = Node::Type::ORIGIN;
        node2->type = Node::Type::DESTINATION;

        //Set coordinates
        node1->x = randlib.unifrnd(-max_latitude, max_latitude);
        node1->y = randlib.unifrnd(-max_longitude, max_longitude);

        node2->x = randlib.unifrnd(-max_latitude, max_latitude);
        node2->y = randlib.unifrnd(-max_longitude, max_longitude);

        node1->load = randlib.randint(1, 1);
        node2->load = -node1->load;

        
        if (verified<=floor(request_num/2)) {
            node1->earliest = randlib.randint(360, 1320);
            node1->latest = node1->earliest+randlib.randint(15,30);
            node2->earliest = 0.0;
            node2->latest = planning_horizon;
            verified++;
        }
        else {
            node1->earliest = 0.0;
            node1->latest = planning_horizon;
            node2->earliest = randlib.randint(0, 720);
            node2->latest = node2->earliest + randlib.randint(15,30);
            verified++;
        }
        node1->service_duration = node1->load;
        node2->service_duration = node1->service_duration;

        node1->maximum_travel_time = randlib.randint(30,40);
        node2->maximum_travel_time = DBL_MAX;

        nodes.push_back(node1);
        nodes.push_back(node2);
        requests.push_back(new Request(node1, node2));
    }

    Node* dummy_node_start = new Node(2 * request_num + 1);
    Node* dummy_node_end = new Node(2 * request_num + 2);
    dummy_node_start->type = Node::Type::START_DEPOT;
    dummy_node_end->type = Node::Type::END_DEPOT;
    dummy_node_start->x = 0;
    dummy_node_start->y = 0;
    dummy_node_end->x = dummy_node_start->x;
    dummy_node_end->y = dummy_node_start->y;
    dummy_node_start->load = 0;
    dummy_node_end->load = 0;
    dummy_node_start->service_duration = 0;
    dummy_node_end->service_duration = 0;
   
    dummy_node_start->earliest = 0.0;
    dummy_node_start->latest = planning_horizon;
    dummy_node_end->earliest = 0.0;
    dummy_node_end->latest = planning_horizon;
    dummy_node_start->maximum_travel_time = DBL_MAX;
    dummy_node_end->maximum_travel_time = DBL_MAX;
   
    nodes.push_back(dummy_node_start);
    nodes.push_back(dummy_node_end);

    //Create and add vehicles
    for (int i = 1; i <= vehicles_num; i++)
    {
        /*We will create both a starting and ending depot for the same location,
        * since we need to represent an earliest departure time when the vehicle starts service
        and a latest arrival time when the platform returns the vehicle to its owner */
        Node* node_start = new Node(2 * request_num + i+2);
        Node* node_end = new Node(2 * request_num + vehicles_num + i+2);
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

        node_start->earliest = 0.0;
        node_start->latest = planning_horizon;
        node_end->earliest = 0.0;
        node_end->latest =planning_horizon;
        node_start->maximum_travel_time = DBL_MAX;
        node_end->maximum_travel_time = DBL_MAX;

        nodes.push_back(node_start);
        nodes.push_back(node_end);
        vehicles.push_back(new EAV(2 * request_num + i,maximumCapacity, maximumBattery, node_start->earliest, node_end->latest,maximumBattery, returnedBatteryPercentage));
    }

    //Create and add charging stations
    for (int i = 1; i <= stations_num; i++)
    {
        Node* node3 = new Node(2 * request_num + 2 *vehicles_num + i+2);
        node3->type = Node::Type::CHARGING_STATION;
        node3->x = randlib.unifrnd(-max_latitude/4, max_latitude/4);
        node3->y = randlib.unifrnd(-max_longitude/4, max_longitude/4);
        node3->load = 0;
        node3->service_duration = 0;
        node3->earliest = 0;
        node3->latest = planning_horizon;
        node3->maximum_travel_time = DBL_MAX;
        nodes.push_back(node3);
        charging_stations.push_back(new CStation(2 * request_num + 2*vehicles_num + i+2, dischargeRate));
    }

    //We do this to represent a request i by (i,n+i) instead of (i,i+1)
    std::sort(nodes.begin(), nodes.end(), [](const Node* n1, const Node* n2) {return n1->id < n2->id; });
    createDistanceMatrix();
    createSimilarityMatrix();
    Preprocessing();
}

void Instance::loadMalheiros(const std::string instance_file_name,double gamma) {
    Horizon = 1440;
    dischargeRate = 0.055;
    maximumBattery = 14.85;
    returnedBatteryPercentage = gamma;
    maxVisitsPerStation = 1;
    std::ifstream file(instance_file_name);

    if (!file.is_open()) {
        std::cerr << "Failed to read file '" << instance_file_name << '\'' << "\n";
        exit(1);
    }

    // Header metadata
    int vehicles_num, requests_num;

    file >> vehicles_num;
    vehicles.reserve(vehicles_num);

    file >> requests_num;
    requests.reserve(requests_num);

    int stations_num=3;
    charging_stations.reserve(3);

    nodes.reserve(2 * requests_num + 2 * vehicles_num + stations_num+2);


    int max_route_duration;
    std::array<int,4> vehicle_capacities;

    Node* node_start = new Node(2 * requests_num + 1);
    Node* node_end = new Node(2 * requests_num + vehicles_num + 2);
    node_start->type = Node::Type::START_DEPOT;
    node_end->type = Node::Type::END_DEPOT;
   
    node_start->x = 0.0;
    node_start->y = 0.0;
    node_end->x = node_start->x;
    node_end->y = node_start->y;
    node_start->load = 0;
    node_end->load = 0;
    node_start->service_duration = 0;
    node_end->service_duration = 0;

    node_start->earliest =0.0;
    node_start->latest = Horizon;
    node_end->earliest = 0.0;
    node_end->latest = Horizon;
    node_start->maximum_travel_time = DBL_MAX;
    node_end->maximum_travel_time = DBL_MAX;

    nodes.push_back(node_start);
    nodes.push_back(node_end);


    // Add vehicles
    for (int i = 1; i <= vehicles_num; i++) {
        file >> max_route_duration;
        for (size_t i = 0; i < 4; i++)
        {
            file >> vehicle_capacities[i];
        }
        int total_capacity = std::accumulate(vehicle_capacities.begin(), vehicle_capacities.end(), 0)/2;
        if (total_capacity > maximumCapacity) maximumCapacity = total_capacity;

        Node* node_start = new Node(2 * requests_num + 2 + i);
        Node* node_end = new Node(2 * requests_num +vehicles_num+2+i);
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
        node_start->service_duration = 0;
        node_end->service_duration = 0;

        node_start->earliest = 0.0;
        node_start->latest = Horizon;
        node_end->earliest = 0.0;
        node_end->latest = Horizon;
        node_start->maximum_travel_time = DBL_MAX;
        node_end->maximum_travel_time = DBL_MAX;

        nodes.push_back(node_start);
        nodes.push_back(node_end);

        vehicles.push_back(new EAV(2 * requests_num + i,total_capacity, maximumBattery,0.0, Horizon,maximumBattery,returnedBatteryPercentage*maximumBattery));
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
            if (!node->maximum_travel_time) node->maximum_travel_time = DBL_MAX;
            std::array<int,4> loads;
            for (int i = 0; i < loads.size(); i++) file >> loads[i];
            node->load = std::accumulate(loads.begin(), loads.end(), 0);

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
        requests.push_back(new Request(nodes[i], nodes[requests_num + i]));
    }

    //Create and add charging stations
    for (int i = 1; i <= stations_num; i++)
    {
        Node* node3 = new Node(2 * requests_num + 2 * vehicles_num + 2 + i);
        node3->type = Node::Type::CHARGING_STATION;
        node3->x = -4.0 + 4*(i-1);
        node3->y = -4.0 + 4*(i-1);
        node3->load = 0;
        node3->service_duration = 0.0;
        node3->earliest = 0.0;
        node3->latest = Horizon;
        node3->maximum_travel_time = DBL_MAX;
        nodes.push_back(node3);
        charging_stations.push_back(new CStation(2 * requests_num + 2 * vehicles_num + 2 + i, dischargeRate));
    }

    createDistanceMatrix();
    createSimilarityMatrix();
    Preprocessing();
}

void Instance::loadInstance(const std::string instance_file_name,double gamma) {
   
    std::ifstream file(instance_file_name);

    if (!file.is_open()) {
        std::cerr << "Failed to read file '" << instance_file_name << '\'' << "\n";
        exit(1);
    }

    // Header metadata
    int vehicles_num, requests_num, start_depots_num, end_depots_num, stations_num, station_copies;

    file >> vehicles_num;
    vehicles.reserve(vehicles_num);

    file >> requests_num;
    requests.reserve(requests_num);

    file >> start_depots_num >> end_depots_num >> stations_num >> station_copies;
    maxVisitsPerStation = station_copies;
    charging_stations.reserve(stations_num);

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
        node->maximum_travel_time = DBL_MAX;

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
        nodes[i]->maximum_travel_time = ride_time;
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

    for (int i = 0; i < vehicles_num; i++) {
        vehicles[i]->minBatteryUponReturn = vehicles[i]->total_battery*gamma;
    }

    double recharge_rate;
    for (int i = 0; i < stations_num; i++) {
        file >> recharge_rate;
        charging_stations[i]->recharge_rate = recharge_rate;
    }
    file >> dischargeRate;
    for (int i = 0; i < 3; i++)
        file >> bestKnownSolutionCost[i]; 
    file.close();

    createDistanceMatrix();
    createSimilarityMatrix();
    Preprocessing();

}

void Instance::createDistanceMatrix()
{
    size_t n = nodes.size();
    distanceMatrix.resize(n);
    forbiddenArcs.resize(n);
    for (int i = 0; i < distanceMatrix.size(); i++) {
        distanceMatrix[i].resize(n);
        forbiddenArcs[i].resize(n);
        for (int j = 0; j < distanceMatrix.size(); j++) {
            distanceMatrix[i][j]=sqrt(
               pow(nodes[j]->x - nodes[i]->x, 2) + pow(nodes[j]->y - nodes[i]->y, 2)
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
    similarRequestsSorted.resize(requests.size());
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
            if (r1->origin->id!=r2->origin->id) similarRequestsSorted[r1->origin->id - 1].push_back(r2);
        }
        std::sort(similarRequestsSorted[r1->origin->id - 1].begin(), similarRequestsSorted[r1->origin->id - 1].end(),
            [&](const Request* req1, const Request* req2)
            {
                return similarity[r1->origin->id - 1][req1->origin->id - 1] <
                       similarity[r1->origin->id - 1][req2->origin->id - 1];
            }
        );
    }
    
}

void Instance::Preprocessing() {

    int n = requests.size();
    for (int i = 0; i < n; i++) {
        if (nodes[i]->latest != 1440.0) {
            nodes[i + n]->earliest = std::max(0.0, nodes[i]->earliest + nodes[i]->service_duration +
                getTravelTime(nodes[i], nodes[i + n]));
            nodes[i + n]->latest = std::min(Horizon, nodes[i]->latest + nodes[i]->service_duration + nodes[i]->maximum_travel_time);

        }
        else {

            nodes[i]->earliest = std::max(0.0, nodes[i + n]->earliest - nodes[i]->service_duration - nodes[i]->maximum_travel_time);
            nodes[i]->latest = std::min(Horizon, nodes[i + n]->latest - nodes[i]->service_duration - inst.getTravelTime(nodes[i], nodes[i + n]));
        }
    }
   

    //Arc Elimination
    avgDistance = 0.0;
    int count = 0;
    size_t length = distanceMatrix.size();
    for (int i = 0; i < length; i++) {
        for (int j = 0; j <length; j++) {
            if (i != j) {

                if ((abs(nodes[i]->load + nodes[j]->load) > inst.maximumCapacity) ||
                    (nodes[i]->isStartingDepot() && nodes[j]->isDestination()) ||
                    (nodes[i]->isOrigin() && nodes[j]->isEndingDepot()) ||
                    (nodes[i]->isOrigin() && nodes[j]->isChargingStation()) ||
                    (nodes[i]->isChargingStation() && nodes[j]->isDestination()) ||
                    (nodes[i]->isDestination() && nodes[j]->isOrigin() && i == j + n) ||
                    (nodes[i]->earliest + nodes[i]->service_duration + getTravelTime(nodes[i], nodes[j]) > nodes[j]->latest))
                {
                   forbiddenArcs[i][j] = 1;
                }
                else if (nodes[i]->isOrigin() && getTravelTime(nodes[i], nodes[j]) + nodes[j]->service_duration + getTravelTime(nodes[j], nodes[i + n]) > nodes[i]->maximum_travel_time) {
                    forbiddenArcs[i][j] = 1;
                    forbiddenArcs[j][n + i]=1;
                }
                if (!forbiddenArcs[i][j]) {
                    avgDistance += distanceMatrix[i][j];
                    count++;
                }

            } 

        }
    }
    avgDistance /= count;
    rejectedRequestPenalties.resize(n+1);
    for (int i = 0; i <= n; i++) rejectedRequestPenalties[i] = i * n* avgDistance;
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
        //printf("%s%i%s%i%s\n", "Request " , r->origin->id , " has " , r->forbidden_vehicles.size() , " forbidden vehicles");
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

Node* Instance::getDepot(EAV* vehicle, bool startingDepot)
{
    return startingDepot? nodes[vehicle->id-1]:nodes[vehicles.size()+vehicle->id-1];
}

double Instance::getTravelTime(Node* n1, Node* n2, bool costMode)
{
    return distanceMatrix[n1->id - 1][n2->id - 1];
}

double Instance::getFuelConsumption(Node* n1, Node* n2)
{
    return dischargeRate * distanceMatrix[n1->id - 1][n2->id - 1];
}

bool Instance::isForbiddenArc(Node* n1, Node* n2) {
    return forbiddenArcs[n1->id-1][n2->id-1];
}