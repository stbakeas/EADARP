#include "Node.h"
Node::Node(int id):id(id){}

bool Node::isOrigin() {
	return type == Type::ORIGIN;
}

bool Node::isDestination() {
	return type == Type::DESTINATION;
}

bool Node::isChargingStation() {
	return type == Type::CHARGING_STATION;
}

bool Node::isStartingDepot() {
	return type == Type::START_DEPOT;
}

bool Node::isEndingDepot() {
	return type == Type::END_DEPOT;
}