#pragma once
#include <vector>
#include <string>
#include "EAV.h"
#include "Request.h"
#include "CStation.h"
#include <unordered_map>
#include <typeinfo>
#include <array>
/**
 * Macro for global access.
 */
#define inst \
        Instance::getUnique()

class Instance
{
private:

    /**
     * Unique Instance.
     */
    static Instance unique;
    Instance() {};

    /*
     * Compute the distance between every pair of nodes and store it in the distance matrix.
     */
    void createDistanceMatrix();

    /*
     * Compute how similar two different requests are.
     */
    void createSimilarityMatrix();

    //Arc Elimination, Vehicle Incompatibility, Request Similarity Matrix
    void Preprocessing();

public:
    std::string name;
    std::vector<Node*> nodes;
    std::vector<EAV*> vehicles;
    std::vector<Request*> requests;
    std::vector<CStation*> charging_stations;
    std::vector<std::vector<double>> distanceMatrix;
    std::vector<std::vector<double>> similarity;
    int controlParameter;
    double maxDistance;
    int numberOfDepots;
    double Horizon;
    double dischargeRate;
    int maximumCapacity = 0.0;
    double returnedBatteryPercentage;
    double maximumBattery;
    ~Instance();
    Instance(const Instance&) = delete;
    void operator=(const Instance&) = delete;
    static Instance& getUnique();
    void loadFromFile(const std::string instance_file_name, int seed);
    void RandomInit(int request_num,int vehicles_num,int stations_num, int max_latitude, int  max_longitude,double planning_horizon, int seed);

    //Get the start depot associated with a vehicle
    Node* getDepot(EAV* vehicle,std::string start_or_end);

    /**
     * Get the request associated with a node.
     */
    Request* getRequest(Node* node);

    /**
     * Get the charging station associated with a node.
     */
    CStation* getChargingStation(Node* node);

    /**
     * Get the travel time between two nodes.
     */
    double getTravelTime(Node* n1, Node* n2);

    /**
     * Get battery consumption between two nodes.
     */
    double getFuelConsumption(Node* n1, Node* n2);

    /*
    * Returns 'true' if the arc starting from n1 and ending at n2 has been removed
    * by the preprocessing algorithm.
    */
    bool isForbiddenArc(Node* n1, Node* n2);
};
