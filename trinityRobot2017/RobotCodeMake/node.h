#ifndef NODE_H
#define NODE_H	

struct Node {
    // I chose to keep node initialization and parent setting as seperate functions 
    Node(int x, int y, int h);
    void setParent(Node* newParent, int dg);

    int x;
    int y;
    int h;
    int g;
    int f;

    Node* parent;
};

#endif
