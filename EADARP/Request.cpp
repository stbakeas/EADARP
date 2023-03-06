#include "Request.h"
#include "Instance.h"

Request::Request(Node* origin,Node* destination) {
	this->origin = origin;
	this->destination = destination;
}