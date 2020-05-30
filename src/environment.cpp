/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

  struct VPoint
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 
    uint16_t ring;                      
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VPoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring))


void ground(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* processor,const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //Applying a filter to downsample the PointCloud  
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);  
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputFiltered=processor->FilterCloud(inputCloud,0.1,Eigen::Vector4f (-10,-4,minPt.z,1), Eigen::Vector4f(10,30,0,1)); 
    
    
    //Segmenting the ground 
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = processor->SegmentPlane(inputFiltered, 1000, 0.3);
    // renderPointCloud(viewer,segmentCloud.first,"not ground",Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"ground",Color(0,1,0));

    pcl::PointCloud<pcl::PointXYZI>::Ptr road (new pcl::PointCloud<pcl::PointXYZI>);


    double z_mean=0;

    for(int i=0;i<segmentCloud.second->points.size();i++)
    {
         z_mean+=segmentCloud.second->points[i].z;
    }

    z_mean=z_mean/segmentCloud.second->points.size();
    std::cerr<<z_mean<<std::endl;

    for(int i=segmentCloud.second->points.size()-1;i>=0;i--)
    {
        if(segmentCloud.second->points[i].z<=z_mean)
        {
            road->points.push_back(segmentCloud.second->points[i]);
        }
    }
    renderPointCloud(viewer,inputFiltered,"not road",Color(1,0,0));
        renderPointCloud(viewer,road,"road",Color(0,1,0));

    


    



    
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
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();


    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/data");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


     while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());


        ground(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}