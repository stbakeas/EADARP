#include "Node.h"
Node::Node(int id):id(id){}

bool Node::isOrigin() noexcept {
	return type == Type::ORIGIN;
}

bool Node::isDestination() noexcept {
	return type == Type::DESTINATION;
}

bool Node::isChargingStation() noexcept {
	return type == Type::CHARGING_STATION;
}

bool Node::isStartingDepot() noexcept {
	return type == Type::START_DEPOT;
}

bool Node::isEndingDepot() noexcept {
	return type == Type::END_DEPOT;
}