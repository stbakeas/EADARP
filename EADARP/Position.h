#pragma once
#include "EAV.h"
#include "CStation.h"
#include <iostream>

struct Position {
	EAV* vehicle;
	int origin_pos;
	int dest_pos;
	double cost;
	CStation* chargingStation;
	Node* node_before_station;

	Position():chargingStation(nullptr),cost(DBL_MAX)
	{};
	

	Position(EAV* vehicle, int origin_pos, int dest_pos,Node* node_before_station,CStation* chargingStation):
		vehicle(vehicle),
		origin_pos(origin_pos),
		dest_pos(dest_pos),
		node_before_station(node_before_station),
		chargingStation(chargingStation),
		cost(0.0){}


};