 #pragma once
#include "Node.h"
class Request {
public:
	Node* origin;
	Node* destination;
	Request() {};
	Request(Node* origin, Node* destination);
	~Request() {};
};