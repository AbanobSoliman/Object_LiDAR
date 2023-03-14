/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}
	
	void insertHelp(Node *&node, uint depth, std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(node == NULL)
		{
		node = new Node(point, id);
		}
		else{
			uint cd = depth % 2; 	
			if (point[cd] < node->point[cd]){
				insertHelp(node->left, depth++, point, id);
			}
			else{
				insertHelp(node->right, depth++, point, id);
			}
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelp(root, 0, point, id);
		

	}

	void searchHelp(std::vector<float> target, float distanceTol, Node *&node, uint depth, std::vector<int>& ids){
	
		if(node != NULL)
		{
			// check the current node (euclidian distance) with target
			if (node->point[0]<=(target[0]+distanceTol)&&node->point[0]>=(target[0]-distanceTol)&&node->point[1]<=(target[1]+distanceTol)&&node->point[1]>=(target[1]-distanceTol)){
				float d = sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[1]-node->point[1])*(target[1]-node->point[1]));
				if (d <= distanceTol){
					ids.push_back(node->id);
				}
			}
			// check the leaves of the tree
			// from left
			if ((target[depth%2] - distanceTol) < node->point[depth%2]){
				searchHelp(target, distanceTol, node->left, depth++, ids);
			}
			// from right
			if ((target[depth%2] + distanceTol) > node->point[depth%2]){
				searchHelp(target, distanceTol, node->right, depth++, ids);
			}
		}
	
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		
		searchHelp(target, distanceTol, root, 0, ids);
		
		return ids;
	}
	

};




