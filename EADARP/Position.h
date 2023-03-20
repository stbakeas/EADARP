#pragma once
#include "EAV.h"
#include "CStation.h"
#include <iostream>

struct Position {
	EAV* vehicle;
	int origin_pos;
	int dest_pos;
	double cost;
	CStation* charging_station;
	int cs_pos;
	double addedDistance;

	Position() {};
	~Position() {};

	Position(EAV* vehicle, int origin_pos, int dest_pos, CStation* charging_station, int cs_pos) {
		this->vehicle = vehicle;
		this->origin_pos = origin_pos;
		this->dest_pos = dest_pos;
		this->charging_station = charging_station;
		this->cs_pos = cs_pos;
		cost = 0.0;
	}

	bool operator<(const Position& a) {
		if (this->vehicle->id < a.vehicle->id) return true;
		else if (this->vehicle->id > a.vehicle->id) return false;
		else {
			if (this->origin_pos < a.origin_pos) return true;
			else if (this->origin_pos > a.origin_pos) return false;
			else {
				if (this->dest_pos < a.dest_pos) return true;
				else return false;
			}
		}
	}

	bool operator!=(const Position& a) {
		if (this->vehicle->id != a.vehicle->id) return true;
		else {
			if (this->origin_pos != a.origin_pos) return true;
			else {
				if (this->dest_pos != a.dest_pos) return true;
				else {
					if (this->charging_station->id != a.charging_station->id) return true;
					else {
						if (this->cs_pos != a.cs_pos) return true;
						return false;
					}
				}

			}
		}
	}

	std::ostream& operator<<(std::ostream& os) {
		os << "(" << this->vehicle->id << " , " << this->origin_pos << " , " << this->dest_pos << ")";
		return os;
	}

	void setAddedDistance(double addedDistance) {
		this->addedDistance = addedDistance;
	}
};