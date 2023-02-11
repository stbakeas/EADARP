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

double Request::SimilarityScore(Request* r)
{
	double timeWindowRelatedness = abs(this->origin->earliest - r->origin->earliest) + abs(this->origin->latest - r->origin->latest) +
		abs(this->destination->latest - r->destination->latest) + abs(this->destination->earliest - r->destination->earliest);
	return inst.getTravelTime(this->origin, r->origin) + inst.getTravelTime(this->destination, r->destination) +
		abs(this->origin->load - r->origin->load) + timeWindowRelatedness;

}
