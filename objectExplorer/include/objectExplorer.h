/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  tanis.mar@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __OBJECTEXPLORER_H__
#define __OBJECTEXPLORER_H__

//Includes
#include <stdio.h>
#include <time.h>
#include <string>
#include <algorithm>

//YARP libs
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>

//ICUB libs
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <iCub/data3D/minBoundBox.h>
#include <iCub/data3D/RGBA.h>

//PCL libs
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

YARP_DECLARE_DEVICES(icubmod)

/**********************************************************/
class ObjectExplorer : public yarp::os::RFModule
{
protected:
    /* variables */ 
    // ports
    yarp::os::BufferedPort<yarp::os::Bottle >                           seedInPort;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox>    meshInPort;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox>    meshOutPort;

    // rpc ports
    yarp::os::RpcServer                 rpcPort;
    yarp::os::RpcClient         		rpcObjRecPort;          //rpc port to communicate with objectReconst module
    yarp::os::RpcClient         		rpcVisualizerPort;      //rpc port to communicate with tool3Dshow module to display pointcloud

    // Drivers
    yarp::dev::PolyDriver               driverG;
    yarp::dev::IGazeControl             *iGaze;

    // config variables
    std::string                         eye;
    std::string                         robot;    
    std::string                         cloudsPath;
    std::string                         cloudName;
    bool                                verbose;
    bool                                normalizePose;

    // module parameters
    bool                                initAlignment;
    bool                                closing;    
    int                                 numCloudsSaved;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_in;       // Last registered pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_temp;   // Merged pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_merged;   // Merged pointcloud    

    /* functions*/
    
    bool                exploreAutomatic(const int regPeriod = 0);
    bool                exploreInteractive();
    bool                getPointCloud();
    bool                alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat);
    void                computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
    void                computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
    bool                showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool                showPointMesh(iCub::data3D::SurfaceMeshWithBoundingBox &meshBottle);
    bool                showPointCloudFromFile(const std::string& fname);

    
    /* Configuration commands */ 
    bool                changeModelName(const std::string& modelname);
    bool                setVerbose(const std::string& verb);

    /* helper functions */
    void                mesh2cloud(const iCub::data3D::SurfaceMeshWithBoundingBox& meshB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void                cloud2mesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, iCub::data3D::SurfaceMeshWithBoundingBox& meshB);
    void                savePointsPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& savename, const bool addNum = true);
    void                savePointsPcd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& savename, const bool addNum = true);
    
public:

   // RF modules overrides    
    bool						configure(yarp::os::ResourceFinder &rf);
    bool						interruptModule();
    bool						close();
    bool						updateModule();
    double						getPeriod();

    bool                        respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};


#endif

