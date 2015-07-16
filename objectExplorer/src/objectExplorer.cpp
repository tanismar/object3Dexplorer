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

#include "objectExplorer.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
  
/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

bool ObjectExplorer::configure(ResourceFinder &rf)
{
    string name = rf.check("name",Value("objectExplorer")).asString().c_str();
    robot = rf.check("robot",Value("icub")).asString().c_str();
    string cloudpath_file = rf.check("clouds",Value("cloudsPath.ini")).asString().c_str();
    rf.findFile(cloudpath_file.c_str());

    ResourceFinder cloudsRF;
    cloudsRF.setContext("object3Dexplorer");
    cloudsRF.setDefaultConfigFile(cloudpath_file.c_str());
    cloudsRF.configure(0,NULL);

    cloudsPath = cloudsRF.find("clouds_path").asString();

    cloudName = rf.check("modelName", Value("cloud")).asString();
    eye = rf.check("camera", Value("left")).asString();
    verbose = rf.check("verbose", Value(true)).asBool();

    //ports
    bool ret = true;
    ret = seedInPort.open(("/"+name+"/seed:i").c_str());	                       // input port to receive data from user
    ret = ret && meshInPort.open(("/"+name+"/mesh:i").c_str());                  // port to receive pointclouds from
    ret = ret && meshOutPort.open(("/"+name+"/mesh:o").c_str());                  // port to receive pointclouds from
    if (!ret){
        printf("\nProblems opening ports\n");
        return false;
    }

    // RPC ports
    bool retRPC = true;
    retRPC = rpcPort.open(("/"+name+"/rpc:i").c_str());
    retRPC = retRPC && rpcObjRecPort.open(("/"+name+"/objrec:rpc").c_str());             // port to send data out for recording
    retRPC = retRPC && rpcVisualizerPort.open(("/"+name+"/visualizer:rpc").c_str());     // port to command the visualizer module
    if (!retRPC){
        printf("\nProblems opening RPC ports\n");
        return false;
    }

    attach(rpcPort);

    printf("\n Opening controllers...\n");

    //Cartesian controllers
    Property optionG("(device gazecontrollerclient)");
    optionG.put("remote","/iKinGazeCtrl");
    optionG.put("local",("/"+name+"/gaze_ctrl").c_str());


    if (!driverG.open(optionG))
        return false;

    driverG.view(iGaze);


    iGaze->setSaccadesStatus(false);
            iGaze->setNeckTrajTime(1.5);
            iGaze->setEyesTrajTime(0.5);
            iGaze->blockEyes(0.0);

    printf("\nInitializing variables... \n");

    closing = false;
    initAlignment = false;
    numCloudsSaved = 0;

    cloud_in = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    cloud_temp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud

    cout << endl << "Configuring done." << endl;
    printf("Base path: %s",cloudsPath.c_str());

    return true;
}

/************************************************************************/
bool ObjectExplorer::interruptModule()
{
    closing = true;

    iGaze->stopControl();

    seedInPort.interrupt();
    meshInPort.interrupt();
    meshOutPort.interrupt();

    rpcPort.interrupt();
    rpcObjRecPort.interrupt();
    rpcVisualizerPort.interrupt();

    return true;
}

/************************************************************************/
bool ObjectExplorer::close()
{
    seedInPort.close();
    meshInPort.close();
    meshOutPort.close();

    rpcPort.close();
    rpcObjRecPort.close();
    rpcVisualizerPort.close();

    driverG.close();
    return true;
}

/************************************************************************/
double ObjectExplorer::getPeriod()
{
    return 0.02;
}

/************************************************************************/
bool ObjectExplorer::updateModule()
{
    return !closing;
}

/************************************************************************/
bool ObjectExplorer::respond(const Bottle &command, Bottle &reply)
{
	/* This method is called when a command string is sent via RPC */
    reply.clear();  // Clear reply bottle

	/* Get command string */
	string receivedCmd = command.get(0).asString().c_str();
	int responseCode;   //Will contain Vocab-encoded response

    if (receivedCmd == "exploreAuto"){
        // Performs automatic exploration, registering images every N seconds, and merging without user confirmation
        bool ok;
        if (command.size() == 2){
            regPeriod = command.get(1).asInt();
            ok = exploreAutomatic(regPeriod);    
        } else {
            ok = exploreAutomatic();
        }
        if (ok){
            reply.addString(" [ack] Exploration successfully finished.");
            return true;
        } else {
            fprintf(stdout,"Couldnt obtain 3D model successfully. \n");
            reply.addString("[nack] Couldnt obtain 3D model successfully.");
            return false;
        }

    }else if (receivedCmd == "exploreInt"){
        // Performs exlporation with feedback, asking the user before merging the obtained point clouds.
        bool ok = exploreInteractive();
        if (ok){
            reply.addString(" [ack] Interactive exploration successfully finished.");
            return true;
        } else {
            fprintf(stdout,"There was an error during the interactive exploration. \n");
            reply.addString("[nack] There was an error during the interactive exploration. ");
            return false;
        }


	}else if (receivedCmd == "get3D"){
        // segment object and get the pointcloud using and save it in file
        bool ok = getPointCloud();
        if (ok) {            
            showPointCloud(cloud_in);
            savePointsPly(cloud_in,cloudName);
            reply.addString(" [ack] 3D registration successfully completed and cloud saved.");
            return true;
        } else {
		    fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
            reply.addString("[nack] Couldnt reconstruct pointcloud. ");
		    return false;
        }

    }else if (receivedCmd == "modelname"){
        // changes the name with which files will be saved by the object-reconstruction module
        string modelname;
        if (command.size() >= 2){
            modelname = command.get(1).asString();
        }else{
            fprintf(stdout,"Please provide a name. \n");
            return false;
        }
        bool ok = changeModelName(modelname);
        if (ok){
            reply.addString(" [ack] Model name successfully changed to ");
            reply.addString(command.get(1).asString());
            return true;
        }else {
            fprintf(stdout,"Couldn't change the name. \n");
            reply.addString("[nack] Couldnt change the name. ");
            return false;
        }

	}else if (receivedCmd == "verbose"){
		bool ok = setVerbose(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Verbose successfully set to ");
            reply.addString(command.get(1).asString());
            return true;
        }
		else {
		    fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
		    return false;
		}

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
		reply.addString("Available commands are:");
        reply.addString("exploreAuto (int)period - automatically gets 3D pointcloud from different perspectives and merges them in a single model.");
        reply.addString("exploreInt - interactively explores the object and asks for confirmation on each registration until a proper 3D model is built.");
		reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module."); 
        reply.addString("modelname (string) - Changes the name with which the pointclouds will be saved.");
        reply.addString("verbose (ON/OFF) - Sets ON/OFF printouts of the program, for debugging or visualization.");        
		reply.addString("help - produces this help.");
		reply.addString("quit - closes the module.");
		reply.addVocab(responseCode);
		return true;

	} else if (receivedCmd == "quit"){
		responseCode = Vocab::encode("ack");
		reply.addVocab(responseCode);
		closing = true;
		return true;
	}
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;

}


/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

/************************************************************************/
bool ObjectExplorer::exploreAutomatic(const int period)
{
    // Register the first pointcloud to initialize
    bool initDone = false;
    cloud_merged->clear();
    string  mergedName = cloudName + "_merged";
    while (!initDone)
    {
        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);
        *cloud_merged = *cloud_in;  //Initialize cloud merged
        savePointsPly(cloud_merged, mergedName,false);
        initDone = true;
        printf("Base cloud initialized \n");

    }

    // Perform Automatic Exploration
    bool explorationDone = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
    while (!explorationDone)        // XXX Code a way to set this flag to true from RPC
    {
        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);

        printf("\n Saving partial registration for later use \n ");
        savePointsPly(cloud_in, cloudName);

        // Merge the last recorded cloud_in with the existing cloud_merged and save on cloud_temp
        Eigen::Matrix4f alignMatrix;
        cloud_aligned->clear();
        alignPointClouds(cloud_in, cloud_merged, cloud_aligned, alignMatrix);
        *cloud_merged += *cloud_aligned;                            // write aligned registration first to a temporal merged

        // Display the merged cloud
        showPointCloud(cloud_merged);
        savePointsPly(cloud_merged, mergedName, false);

        Time::delay(period);

    }

    // Visualize merged pointcloud
    printf("Exploration finished, returning control. \n");
    return true;

}

/************************************************************************/
bool ObjectExplorer::exploreInteractive()
{   // Explore the object and incrementally check registration and add merge pointclouds if correct

    // Register the first pointcloud to initialize
    bool initDone = false;
    cloud_merged->clear();
    string  mergedName = cloudName + "_merged";
    while (!initDone)
    {

        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);

        printf("Is the registered cloud clean? (y/n)? \n");
        string answerInit;
        cin >> answerInit;
        if ((answerInit == "y")||(answerInit == "Y"))
        {
            *cloud_merged = *cloud_in;  //Initialize cloud merged
            *cloud_temp = *cloud_in;    //Initialize auxiliary cloud on which temporal merges will be shown before confirmation
            savePointsPcd(cloud_merged, mergedName,false);
            initDone = true;
            printf("Base cloud initialized \n");
        }else {
            printf(" Try again \n");
        }
    }

    // Perform interactive exploration
    bool explorationDone = false;
    while (!explorationDone)
    {
        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);

        // If it is ok, do the merge and display again.
        printf("Is the registered cloud clean? (y/n)? \n >> ");
        string answerReg;
        cin >> answerReg;
        if ((answerReg == "y")||(answerReg == "Y"))
        {
            printf("\n Saving partial registration for later use \n ");
            savePointsPcd(cloud_in, cloudName);

            // If the cloud is clean, merge the last recorded cloud_in with the existing cloud_merged and save on cloud_temp
            Eigen::Matrix4f alignMatrix;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
            alignPointClouds(cloud_in, cloud_merged, cloud_aligned, alignMatrix);
            *cloud_temp += *cloud_aligned;                            // write aligned registration first to a temporal merged

            // Display the merged cloud
            showPointCloud(cloud_temp);

            // If merge is good, update model
            // XXX offer three options
            // - Yes, its is good, save (y)
            // - No, it sucks, go for another registration (n)
            // - Retry merging with different parameters (r)
            // XXX this last option implies making some merging parameters tweakable ( use of FPFH / ICP, iterations, min dist, etc).
            printf("Is the merged cloud clean? (y/n)? \n >> ");
            string answerMerge;
            cin >> answerMerge;
            if ((answerMerge == "y")||(answerMerge == "Y"))
            {
                cloud_merged->clear();
                *cloud_merged = *cloud_temp;
                printf(" The model has been updated \n");

                // Ask if more registrations need to be made, and if so, loop again. Otherwise, break.
                printf(" Should I take another registration? (y/n) \n");
                string answerModel;
                cin >> answerModel;
                if ((answerModel == "y")||(answerModel == "Y"))
                {
                    printf(" Model not finished, continuing with exploration \n");
                } else {
                    savePointsPcd(cloud_merged, mergedName, false);
                    printf(" Final model saved as %s, finishing exploration \n", mergedName.c_str());
                    explorationDone = true;
                }
            } else {
                printf("\n Ignoring merge, continue with exploration \n");
                *cloud_temp = *cloud_merged;
            }
        } else {
            printf("\n Unproper registration, try again \n");
        }
    }

    // Visualize merged pointcloud
    printf("Exploration finished, returning control. \n");
    return true;
}


/************************************************************************/
bool ObjectExplorer::getPointCloud()
{
    cloud_in->clear();   // clear receiving cloud

    // read coordinates from yarpview
    if (verbose){printf("Please click on seed point from the Segmentation viewer. \n");}
    Bottle *seedIn = seedInPort.read(true);	//waits until it receives coordinates
    int u = seedIn->get(0).asInt();
    int v = seedIn->get(1).asInt();
    if (verbose){cout << "Retrieving blob from u: "<< u << ", v: "<< v << endl;	}

    // send image blob coordinates as rpc to objectRec to seed the cloud
    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addInt(u);
    cmdOR.addInt(v);
    rpcObjRecPort.write(cmdOR,replyOR);

    // requests 3D reconstruction to objectReconst module
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("3Drec");
    cmdOR.addString("visualize");
    rpcObjRecPort.write(cmdOR,replyOR);

    // read the cloud from the objectReconst output port
    iCub::data3D::SurfaceMeshWithBoundingBox *cloudMesh = meshInPort.read(true);	//waits until it receives coordinates
    if (cloudMesh!=NULL){
        //showPointMesh(*cloudMesh);
        if (verbose){	printf("Cloud read from port \n");	}
        mesh2cloud(*cloudMesh,cloud_in);
    } else{
        if (verbose){	printf("Couldnt read returned cloud \n");	}
        return -1;
    }

    // Apply some filtering to clean the cloud
    // Process the cloud by removing distant points ...
    const float depth_limit = 0.5;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, depth_limit);
    pass.filter (*cloud_in);

     // ... and removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
    sor.setStddevMulThresh (1.0);
    sor.setInputCloud (cloud_in);
    sor.setMeanK(cloud_in->size()/2);
    sor.filter (*cloud_in);


    if (verbose){ cout << " Cloud of size " << cloud_in->points.size() << " obtained from 3D reconstruction" << endl;}
    return true;
}


/************************************************************************/
bool ObjectExplorer::alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat)
{
    // --------------------------- Use FPFH features for initial alignment ---------------------------------
    printf("Applying FPFH alignment... \n");
    if (verbose){printf("Defining features... \n");}
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target (new pcl::PointCloud<pcl::FPFHSignature33>);

    // Feature computation
    if (verbose){printf("Computing features... \n");}
    computeLocalFeatures(cloud_from, features);
    computeLocalFeatures(cloud_to, features_target);

    // Perform initial alignment
    if (verbose){printf("Setting Initial alignment parameters \n");}
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
    sac_ia.setMinSampleDistance (0.05f);
    sac_ia.setMaxCorrespondenceDistance (0.01f*0.01f);
    sac_ia.setMaximumIterations (500);

    if (verbose){printf("Adding source cloud\n");}
    sac_ia.setInputTarget(cloud_to);
    sac_ia.setTargetFeatures (features_target);

    if (verbose){printf("Adding target cloud\n");}
    sac_ia.setInputSource(cloud_from);
    sac_ia.setSourceFeatures (features);

    if (verbose){printf("Aligning clouds\n");}
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_init_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
    sac_ia.align(*cloud_init_aligned);

    cout << "FPFH has converged:" << sac_ia.hasConverged() << " score: " << sac_ia.getFitnessScore() << endl;

    if (verbose){printf("Getting alineation matrix\n");}
    Eigen::Matrix4f initAlignMat = sac_ia.getFinalTransformation();

    // -------------------- Apply good old ICP registration for fine alginment --------------------------------

    printf("Applying ICP fine lignment... \n");
    // The Iterative Closest Point algorithm
    if (verbose){printf("Setting ICP parameters \n");}
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    //ICP algorithm
    if (verbose){printf("Adding source cloud \n");}
    icp.setInputSource(cloud_init_aligned);
    if (verbose){printf("Adding target cloud \n");}
    icp.setInputTarget(cloud_to);
    if (verbose){printf("Aligning clouds\n");}
    icp.align(*cloud_aligned);

    cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;

    if (verbose){printf("Getting alineation matrix\n");}
    cout << icp.getFinalTransformation() << endl;
    if (verbose){printf("Clouds aligned! \n");}
    transfMat = icp.getFinalTransformation();

    return true;
}

void ObjectExplorer::computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    pcl::search::KdTree<pcl::PointXYZRGB> ::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    computeSurfaceNormals(cloud, normals);

    printf("Computing Local Features\n");
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.02f);
    fpfh_est.compute(*features);
}


void ObjectExplorer::computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    printf("Computing Surface Normals\n");
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch(0.02f);
    norm_est.compute(*normals);
}

/************************************************************************/
bool ObjectExplorer::showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    iCub::data3D::SurfaceMeshWithBoundingBox &meshBottle = meshOutPort.prepare();
    cloud2mesh(cloud, meshBottle);
    if (verbose){printf("Sending out cloud. \n");}
    meshOutPort.write();
    return true;
}

bool ObjectExplorer::showPointMesh(iCub::data3D::SurfaceMeshWithBoundingBox& meshBottle)
{
    meshBottle = meshOutPort.prepare();
    if (verbose){printf("Sending out mesh. \n");}
    meshOutPort.write();
    return true;
}

/************************************************************************/
bool ObjectExplorer::showPointCloudFromFile(const string& fname)
{

    Bottle cmdVis, replyVis;
    cmdVis.clear();	replyVis.clear();
    cmdVis.addString("name");
    cmdVis.addString(fname);
    rpcVisualizerPort.write(cmdVis,replyVis);

    // Sends an RPC command to the tool3Dshow module to display the merged pointcloud on the visualizer
    cmdVis.clear();	replyVis.clear();
    cmdVis.addString("show");
    rpcVisualizerPort.write(cmdVis,replyVis);

    return true;
}


/************************************************************************/
bool ObjectExplorer::changeModelName(const string& modelname)
{
    // Changes the name with which the pointclouds will be saved and read
    cloudName = modelname;

    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("name");
    cmdOR.addString(modelname);
    rpcObjRecPort.write(cmdOR,replyOR);

    printf("Name changed to %s.\n", modelname.c_str());
return true;
}

/*************************** -Conf Commands- ******************************/
bool ObjectExplorer::setVerbose(const string& verb)
{
    if (verb == "ON"){
        verbose = true;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    } else if (verb == "OFF"){
        verbose = false;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    }
    return false;
}


/*************************** -Helper functions- ******************************/

void ObjectExplorer::mesh2cloud(const iCub::data3D::SurfaceMeshWithBoundingBox& meshB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{   // Converts mesh from a bottle into pcl pointcloud.
    for (size_t i = 0; i<meshB.mesh.points.size(); ++i)
    {
        pcl::PointXYZRGB pointrgb;
        pointrgb.x=meshB.mesh.points.at(i).x;
        pointrgb.y=meshB.mesh.points.at(i).y;
        pointrgb.z=meshB.mesh.points.at(i).z;
        if (i<meshB.mesh.rgbColour.size())
        {
            int32_t rgb= meshB.mesh.rgbColour.at(i).rgba;
            pointrgb.rgba=rgb;
            pointrgb.r = (rgb >> 16) & 0x0000ff;
            pointrgb.g = (rgb >> 8)  & 0x0000ff;
            pointrgb.b = (rgb)       & 0x0000ff;
        }
        else
            pointrgb.rgb=0;

        cloud->push_back(pointrgb);
    }
    if (verbose){cout << "Mesh formatted as Point Cloud of size " << cloud->points.size() << endl;}
}


void ObjectExplorer::cloud2mesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, iCub::data3D::SurfaceMeshWithBoundingBox& meshB)
{   // Converts pointcloud to surfaceMesh bottle.

    if (verbose){printf("Transforming cloud to mesh to be sent out... ");}
    meshB.mesh.points.clear();
    meshB.mesh.rgbColour.clear();
    meshB.mesh.meshName = cloudName;
    for (unsigned int i=0; i<cloud->width; i++)
    {
        meshB.mesh.points.push_back(iCub::data3D::PointXYZ(cloud->at(i).x,cloud->at(i).y, cloud->at(i).z));
        meshB.mesh.rgbColour.push_back(iCub::data3D::RGBA(cloud->at(i).rgba));
    }
    //if (verbose){printf("\n Computing minimum BB.\n");}
    iCub::data3D::BoundingBox BB = iCub::data3D::MinimumBoundingBox::getMinimumBoundingBox(cloud);
    meshB.boundingBox = BB.getBoundingBox();
    if (verbose){printf("\n Mesh obtained from cloud. \n");}
    return;
}


/************************************************************************/
void ObjectExplorer::savePointsPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& savename, const bool addNum)
{
    stringstream s;
    s.str("");
    if (addNum){
        s << cloudsPath + "/" + savename.c_str() << numCloudsSaved;
        numCloudsSaved++;
    } else {
        s << cloudsPath + "/" + savename.c_str();
    }

    string filename = s.str();
    string filenameNumb = filename+".ply";
    ofstream plyfile;
    plyfile.open(filenameNumb.c_str());
    plyfile << "ply\n";
    plyfile << "format ascii 1.0\n";
    plyfile << "element vertex " << cloud->width <<"\n";
    plyfile << "property float x\n";
    plyfile << "property float y\n";
    plyfile << "property float z\n";
    plyfile << "property uchar diffuse_red\n";
    plyfile << "property uchar diffuse_green\n";
    plyfile << "property uchar diffuse_blue\n";
    plyfile << "end_header\n";

    for (unsigned int i=0; i<cloud->width; i++)
        plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";

    plyfile.close();

    fprintf(stdout, "Writing finished\n");
}


void ObjectExplorer::savePointsPcd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& savename, const bool addNum)
{
    stringstream s;
    s.str("");
    if (addNum){
        s << cloudsPath + "/" + savename.c_str() << numCloudsSaved;
        numCloudsSaved++;
    } else {
        s << cloudsPath + "/" + savename.c_str();
    }

    string filename = s.str() + ".pcd";

    pcl::io::savePCDFileASCII (filename, *cloud);

}

/************************************************************************/
/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setDefaultContext("object3Dexplorer");
    rf.setDefaultConfigFile("objectExplorer.ini");
    rf.setVerbose(true);
    rf.configure(argc,argv);

    ObjectExplorer objectExplorer;

    cout<< endl <<"Configure module..."<<endl;
    objectExplorer.configure(rf);
    cout<< endl << "Start module..."<<endl;
    objectExplorer.runModule();

    cout<<"Main returning..."<<endl;

    return 0;
}


