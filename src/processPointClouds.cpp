// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, Eigen::Vector4f minPoint_ROOF, Eigen::Vector4f maxPoint_ROOF)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // 1 - Voxel Grid Filter (down sampling)
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered1(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes); // float in meters
    sor.filter (*cloud_filtered1);
    std::cout << "VoxGrd Filtered PCD has " << cloud_filtered1->points.size () << " data points " << std::endl;
    
    // 2 - ROI Filter (to remove car roof points)
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    // a - apply to all cloud
    pcl::CropBox<PointT> roi(true);
    roi.setInputCloud (cloud_filtered1);
    roi.setMin (minPoint);
    roi.setMax (maxPoint);
    roi.filter (*cloud_filtered2);
    // b - apply to roof cloud
    std::vector<int> roof_idx;
    pcl::CropBox<PointT> roi_roof(true);
    roi_roof.setInputCloud (cloud_filtered2);
    roi_roof.setMin (minPoint_ROOF);
    roi_roof.setMax (maxPoint_ROOF);
    roi_roof.filter (roof_idx);
    // Extract the outlier indices from the ROI filtered
    typename pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    for (auto point : roof_idx)
        inliers->indices.push_back(point);
    // Extract the inliers
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers);
    extract.setNegative (true);  // remove these points (they are pcd outliers, and inliers to the roof box)
    extract.filter (*cloud_filtered2);
    
    std::cout << "ROI Filtered PCD has " << cloud_filtered2->points.size () << " data points " << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered2;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>), planeCloud (new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given point cloud." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
    if (cloud->points.size() == 0){
	std::cerr << "Cloud has no points to segment!" << std::endl;
    	pcl::PointIndices::Ptr inlierss (new pcl::PointIndices());
    	inlierss->indices.push_back(0);
    	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierss,cloud);
        return segResult;
    }
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.  
    
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    float a,b,c,d,D;
    int i;
    auto itr = inliersResult.begin();
    PointT p1,p2,p3;

    while (maxIterations--){
    
	    std::unordered_set<int> inliers;
	    while (inliers.size() < 3)
		inliers.insert(rand() % (cloud->points.size()));
		    
	    itr = inliers.begin();
	    p1 = cloud->points[*itr];
	    itr++;
	    p2 = cloud->points[*itr];
	    itr++;
	    p3 = cloud->points[*itr];
	    a = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
	    b = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
	    c = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
	    D = -(a*p1.x+b*p1.y+c*p1.z);
	    
	    for (i = 0; i < cloud->points.size(); i++){
	    
		if (inliers.count(i)>0)
		    continue;
	    
		d = fabs(a*cloud->points[i].x + b*cloud->points[i].y + c*cloud->points[i].z + D) / sqrt(a*a + b*b + c*c);
		
		if (d<=distanceThreshold){
		    inliers.insert(i);
		}
	    
	    }
	    
	    if (inliers.size() > inliersResult.size()){
		inliersResult = inliers;
	    }
    
    }
    
    pcl::PointIndices::Ptr inlierss (new pcl::PointIndices());
    for (auto idx : inliersResult)
        inlierss->indices.push_back(idx);
    
    if (inlierss->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given point cloud." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "our ransac plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierss,cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int i, const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol, std::vector<bool>& processed, std::vector<int>& cluster)
{
	// TODO: Fill out this function to return list of indices for each cluster
	processed[i] = true;
	cluster.push_back(i);
	std::vector<int> nearby = tree->search(points[i],distanceTol);
	for (auto idx_NB : nearby){
		if (!processed[idx_NB]){
			Proximity(idx_NB, points, tree, distanceTol, processed, cluster);
		}
	}
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	int i = 0;
	while (i < points.size()){
		if (processed[i]){
			i++;
			continue;
		}
		std::vector<int> cluster;		
		Proximity(i, points, tree, distanceTol, processed, cluster);
		clusters.push_back(cluster);
	}
	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // My clustering function implementation
    std::vector<std::vector<float>> points;  // extract cloud points
    for (int i=0; i<cloud->points.size(); i++)
    	points.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    
    KdTree* tree = new KdTree;
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i);
    	
    std::vector<std::vector<int>> clusters_idx = euclideanCluster(points, tree, clusterTolerance);
    int clusterId = 0;
    for(std::vector<int> cluster : clusters_idx){
    	if (cluster.size()<=minSize or cluster.size()>=maxSize){
	    continue;
    	}
	typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
	for(int indice: cluster)
		clusterCloud->points.push_back(cloud->points[indice]);
	clusters.push_back(clusterCloud);
	++clusterId;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    for (const auto& cluster : cluster_indices)
    {
      typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
      for (const auto& idx : cluster.indices) {
        cluster_cloud->points.push_back(cloud->points[idx]);
      }
      cluster_cloud->width = cluster_cloud->size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;
      clusters.push_back(cluster_cloud);
     }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
