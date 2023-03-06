 #pragma once
#include "Node.h"
#include <unordered_set>
#include "EAV.h"
#include <map>

class Request {
public:
	Node* origin;
	Node* destination;
	std::unordered_set<int> forbidden_vehicles;
	Request() {};
	Request(Node* origin, Node* destination);
	~Request() {};
};