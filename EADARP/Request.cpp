#include "Request.h"
Request::Request(Node* origin,Node* destination,Direction direction) {
	this->origin = origin;
	this->destination = destination;
	this->direction = direction;
	this->reward = 0.0;
}
