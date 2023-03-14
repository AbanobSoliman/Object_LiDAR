// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <boost/filesystem.hpp>
#include <chrono>
#include "render/box.h"
#include <unordered_set>



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
	uint k = 3;

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
			uint cd = depth % k; 	
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
			if (node->point[0]<=(target[0]+distanceTol)&&node->point[0]>=(target[0]-distanceTol)&&node->point[1]<=(target[1]+distanceTol)&&node->point[1]>=(target[1]-distanceTol)&&node->point[2]<=(target[2]+distanceTol)&&node->point[2]>=(target[2]-distanceTol)){
				float d = sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[1]-node->point[1])*(target[1]-node->point[1])+(target[2]-node->point[2])*(target[2]-node->point[2]));
				if (d <= distanceTol){
					ids.push_back(node->id);
				}
			}
			// check the leaves of the tree
			// from left
			if ((target[depth%k] - distanceTol) < node->point[depth%k]){
				searchHelp(target, distanceTol, node->left, depth++, ids);
			}
			// from right
			if ((target[depth%k] + distanceTol) > node->point[depth%k]){
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


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, Eigen::Vector4f minPoint_ROOF, Eigen::Vector4f maxPoint_ROOF);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold); // my segmentation function
    
    // cluster helper functions
    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol);
    void Proximity(int i, const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol, std::vector<bool>& processed, std::vector<int>& cluster);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize); // my clustering function

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};

#endif /* PROCESSPOINTCLOUDS_H_ */
