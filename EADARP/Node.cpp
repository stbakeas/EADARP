#include "Node.h"
Node::Node(int id):id(id){}

bool Node::isOrigin() const noexcept {
	return type == Type::ORIGIN;
}

bool Node::isDestination() const noexcept {
	return type == Type::DESTINATION;
}

bool Node::isChargingStation() const noexcept {
	return type == Type::CHARGING_STATION;
}

bool Node::isStartingDepot() const noexcept {
	return type == Type::START_DEPOT;
}

bool Node::isEndingDepot() const noexcept {
	return type == Type::END_DEPOT;
}