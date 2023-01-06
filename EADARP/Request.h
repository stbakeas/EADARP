 #pragma once
#include "Node.h"
#include <unordered_set>
#include "EAV.h"

class Request {
public:
	enum class Direction{ INBOUND,OUTBOUND };
	Direction direction;
	Node* origin;
	Node* destination;
	int reward;
	std::unordered_set<int> forbidden_vehicles[4];
	Request() {};
	Request(Node* origin, Node* destination,Direction direction);
	~Request() {};
};