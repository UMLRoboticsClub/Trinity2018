#ifndef NODE_H
#define NODE_H	

using namespace std;

class Node {

public:
	// I chose to keep node initialization and parent setting as seperate functions 
	Node(int x, int y, int h);
	void setParent(Node* newParent, int dg);

	Node* getParent(void);

	// standard getters
	int getX()const;
	int getY()const;
	int getG()const;
	int getF()const;

private:

	int x;
	int y;
	int h;
	int g;
	int f;

	Node* parent;
};
#endif // !NODE_H
