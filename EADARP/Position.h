#pragma once
#include "EAV.h"
#include "CStation.h"
#include <iostream>

struct Position {
	EAV* vehicle;
	int origin_pos;
	int dest_pos;
	double cost;
	std::vector<std::pair<CStation*, Node*>> cs_pos;
	double addedDistance;

	Position() {};
	~Position() {};

	Position(EAV* vehicle, int origin_pos, int dest_pos):
		vehicle(vehicle),
		origin_pos(origin_pos),
		dest_pos(dest_pos),
		cost(0.0){}

	std::ostream& operator<<(std::ostream& os) {
		os << "(" << this->vehicle->id << " , " << this->origin_pos << " , " << this->dest_pos << ")";
		return os;
	}

	void setAddedDistance(double addedDistance) {
		this->addedDistance = addedDistance;
	}
};