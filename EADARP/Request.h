 #pragma once
#include "Node.h"
#include <unordered_map>
#include "EAV.h"
#include <map>

class Request {
public:
	enum class Direction{ INBOUND,OUTBOUND };
	Direction direction;
	Node* origin;
	Node* destination;
	int reward;
	std::unordered_map<int,double> forbidden_vehicles[4];
	std::map<double, double> percentages[4];
	Request() {};
	Request(Node* origin, Node* destination,Direction direction);
	Node* getCriticalVertex();
	double SimilarityScore(Request* r);
	~Request() {};
};