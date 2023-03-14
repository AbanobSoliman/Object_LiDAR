/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

template<typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<PointT>* pointProcessorI, const typename pcl::PointCloud<PointT>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  float leaf = 0.25;
  Eigen::Vector4f minP_all(-10.0, -5.0, -2.0, 1.0); // the big box min size around the AGV
  Eigen::Vector4f maxP_all(30.0, 8.0, 1.0, 1.0);  // the big box max size around the AGV
  Eigen::Vector4f minP_roof(-1.5, -1.7, -1, 1.0); // static for all pcds
  Eigen::Vector4f maxP_roof(2.6, 1.7, -0.4, 1.0); // static for all pcds
  typename pcl::PointCloud<PointT>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, leaf, minP_all, maxP_all, minP_roof, maxP_roof);
  //renderPointCloud(viewer,filterCloud,"filterCloud");
  
  // my segmentation function
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segment(pointProcessorI->RansacPlane(filterCloud, 25, 0.3));
  //renderPointCloud(viewer,segment.first,"Obstacles", Color(1,0,0));
  
  // my clustering function
  std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstClusters1(pointProcessorI->EuclideanCluster(segment.first, 0.85, 3, 1000)); // cluster objects in 3 categories
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for(typename pcl::PointCloud<PointT>::Ptr cluster : ObstClusters1)
  {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]); // cars in red
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer,box,clusterId);
    ++clusterId;
  }
  renderPointCloud(viewer,segment.second,"Road", Color(0,1,0));  // segmented plane (road) in green
  
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0.0); // on heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (lidar->scan());  // on stack
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "Hi PCL", Color(1,0,0));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pcl_process_H = new ProcessPointClouds<pcl::PointXYZ>(); // on heap
    ProcessPointClouds<pcl::PointXYZ> pcl_process_S; // on stack
    
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> heap_test(pcl_process_H->SegmentPlane(cloud, 100, 0.2)); // 6 msec
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> stack_test(pcl_process_S.SegmentPlane(cloud, 100, 0.2)); // 0 msec
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> heap_test(pcl_process_H->RansacPlane(cloud, 100, 0.2)); // 22 msec
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmt_test(pcl_process_S.RansacPlane(cloud, 100, 0.2)); // 22 msec
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clust_test(pcl_process_S.EuclideanCluster(segmt_test.first, 1.0, 3, 30)); //  msec
    
    //renderPointCloud(viewer, heap_test.first, "Obstacles", Color(1,0,0));
    //renderPointCloud(viewer, heap_test.second, "Planes", Color(0,1,0));
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clust_test)
    {
      std::cout << "cluster size ";
      pcl_process_S.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box = pcl_process_S.BoundingBox(cluster);
      renderBox(viewer,box,clusterId);
      ++clusterId;
    }
    

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment New" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
          streamIterator = stream.begin();
        
        viewer->spinOnce ();
    } 
}
