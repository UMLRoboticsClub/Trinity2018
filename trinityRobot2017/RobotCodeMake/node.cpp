#include "node.h"
#include <cstddef> // for NULL

Node::Node(int x, int y, int h) {
	this->x = x;
	this->y = y;
	this->h = h;
}

int Node::getX()const {
	return this->x;
}
int Node::getY()const {
	return this->y;
}
int Node::getG()const {
	return this->g;
}
int Node::getF()const {
	return this->f;
}

void Node::setParent(Node* newParent, int dg) {
	this->parent = newParent;
	if (this->parent != NULL) {
		this->g = this->parent->getG() + dg;
	}
	else {
		// if this is first node
		this->g = dg;
	}
	this->f = this->g + this->h;
}


Node* Node::getParent(void) {
	return this->parent;
}