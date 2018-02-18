#include "node.h"

Node::Node(int x, int y, int h):
x(x), y(y), h(h){}

void Node::setParent(Node *newParent, int dg) {
    parent = newParent;
    if (parent != nullptr) {
        g = parent->g + dg;
    } else {
        // if this is first node
        g = dg;
    }
    f = g + h;
}
