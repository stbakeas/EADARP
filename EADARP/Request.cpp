#include "Request.h"
#include "Instance.h"

Request::Request(Node* origin,Node* destination,Direction direction) {
	this->origin = origin;
	this->destination = destination;
	this->direction = direction;
	this->reward = 0.0;
}

Node* Request::getCriticalVertex() {
	return direction == Request::Direction::INBOUND ? origin : destination;
}