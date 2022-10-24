#include "Node.h"
Node::Node(int id) {
	this->id = id;
}

bool Node::isOrigin() {
	return type == Type::ORIGIN;
}

bool Node::isDestination() {
	return type == Type::DESTINATION;
}

bool Node::isChargingStation() {
	return type == Type::CHARGING_STATION;
}

bool Node::isDepot() {
	return type == Type::DEPOT;
}