#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <string>
#include <fstream>
#include <chrono>
#include "aruco_ros_pose//markerpose.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include "pnp_solver/my_pnp.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "aruco_ros_pose/inner_corners.hpp"



using namespace std;
using namespace cv;

struct PosePnPResult
{
    ros::Time time;
    cv::Vec3d vt;
    cv::Vec3d rt;
};



class aruco_localization
{
    private:
    //@ ros Publisher and Subscriber
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    // image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher marker_pub_;

    // ROS-Block for Marker-Visualization
    uint32_t shape = visualization_msgs::Marker::CUBE;

    //@ aruco dictionary and aruco detection parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

    //@ camera intrinsics
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    //@ Side length of marker
    double MarkerSize = 0.100;

    //@ CamID if no video_file is given
    int camId = 0;
    
    //Path to file of a video with markers and path where the estimated data should be saved
    cv::String video = "/home/hkh/catkin_ws/src/aruco_ros_pose/Videos_Huang/newtest.mp4";
    std::string save_path = "/home/hkh/kehui/data";

    //@ Variable if InnerCorners should be detected or not (Adjust InnerCorners in "detector_params.yaml")
    int InnerCorners;
    //@ Object instance: load marker map
    Markerpose marker;
     
    //@ Transform vehicle to camera
    std::vector<double> Transform_vc = {0, 0, 0.275, 1.5708, 3.14159265, 1.5708};

    public:
    //@ Constructor: use video topic for initialization
	aruco_localization();


    //@ read detector parameters from "detector_params.yaml"
	void readDetectorParameters(cv::Ptr<aruco::DetectorParameters> &params);

    //@ Generate 2d-3d correspondences as PnP input according to the detection result
    //@@ input:: detectedIds:The ids of the detected markers, detectedCorners: pixel coordinates of marker corner
    //@@ output:: objPoints:marker corner coordinates in 3D world, imgPoints: The coordinates of the marker corner in the image
	void getObjectAndImagePoints(InputArrayOfArrays detectedCorners,
    		InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints);



    void Readcameracalib(Mat &camMatrix, Mat &distCoeffs);
    void pose();


};


aruco_localization::aruco_localization() : it_(nh_)
{

    image_pub_ = it_.advertise("camera/image", 1);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("vehicle_pose", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    parameters = cv::aruco::DetectorParameters::create();


    //@ read camera intrinsics parameters from "camera_calibration.yaml"
    Readcameracalib(cameraMatrix, distCoeffs);

    //@ set all detection parameters from "detector_params.yaml"
    readDetectorParameters(parameters);
    // fout.open("/home/likun/catkin_ws/src/localization_with_artrack_cv/data/time.txt");
    pose();
}



void aruco_localization::Readcameracalib(Mat &cameraMatrix, Mat &distCoeffs) {
    std::string calib_path = ros::package::getPath("aruco_ros_pose");
    calib_path = calib_path + "/config/camera_calibration.yaml";
    FileStorage fs(calib_path, FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_ERROR("Invalid camera calibration file");
        return;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    cerr << "1" << endl;
}


void aruco_localization::readDetectorParameters(Ptr<aruco::DetectorParameters> &params) {
    std::string paras_path = ros::package::getPath("aruco_ros_pose");
    paras_path = paras_path + "/config/detector_params.yaml";
    FileStorage fs(paras_path, FileStorage::READ);

    std::string path = ros::package::getPath("aruco_ros_pose");

    if(!fs.isOpened())
    {
        ROS_ERROR("Invalid detector parameters file");
        return;
    }
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    fs["InnerCorners"] >> InnerCorners;
    fs.release();
    cerr << "2" << endl;
}



void aruco_localization::getObjectAndImagePoints(InputArrayOfArrays detectedCorners,
InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints) {
    size_t nDetectedMarkers = detectedIds.total();

    vector< Point3f > objPnts;
    objPnts.reserve(nDetectedMarkers);
    vector< Point2f > imgPnts;
    imgPnts.reserve(nDetectedMarkers);
    cerr << "3" << endl;

    for(unsigned int i = 0; i < nDetectedMarkers; i++) 
    {   
        std::vector<Vec3f> markercornersinworld;
        int id = detectedIds.getMat().ptr< int >(0)[i];
        marker.getObjectPointsInWorld(id,markercornersinworld);

        if (!markercornersinworld.empty())
        {
            for (int j = 0; j < 4; ++j)
            {
                objPnts.push_back(Point3d(markercornersinworld[j][0],markercornersinworld[j][1],markercornersinworld[j][2]));
                imgPnts.push_back(detectedCorners.getMat(i).ptr< Point2f >(0)[j]);
            }
            
        }

    }
    //@ create output
    Mat(objPnts).copyTo(objPoints);
    Mat(imgPnts).copyTo(imgPoints);
    cerr << "3" << endl;
}


void aruco_localization::pose()
{   

    // Look if video-file is given
    VideoCapture inputVideo;
    int waitTime;
    // If a file is given, then open it
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 10;
    } 
    // If no file is given, then open camera (of laptop or webcam or else)
    else{
        inputVideo.open(camId);
        waitTime = 10;
    }

    // Frame for content
    cv::Mat frame;
    // Initialize image which can be published
    sensor_msgs::ImagePtr img;
    // Variable for publishing the vehicle position
    geometry_msgs::PoseStamped VehiclePoseSt;
    // Variable for default image and edited image
    Mat image, imageCopy;


    // While video or cam is playing the loop continues
    while(inputVideo.grab())
    {

    inputVideo.retrieve(image);

    //@ Variable for detected ids
    vector< int > ids;
    //@ Vector with detected corners in px
    vector< vector< Point2f >> corners, rejected;
    //@ Rotation- and translationVector for transformation between camera- and markerCoordinateSystem (after estimatePoseSingleMarker)
    vector < Vec3d > rvecs, tvecs;
    //@ Rotation- and translationVector for transformation between world- and markerCoordinateSystem (after SolvePnP)
    cv::Vec3d rvec, tvec;

    // Masage to show the markerCube in image
    visualization_msgs::Marker marker_visual;

    Mat objPoints, imgPoints;

    // Function to detect markers and return their corners and IDs
    aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);

    // Variable for saving the time when a marker is detected
    const auto p1 = std::chrono::system_clock::now();
    std::string duration = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count());

    // Copy original image for editing the image and dont loose original
    image.copyTo(imageCopy);

    // Variable for publishing data
    static tf2_ros::TransformBroadcaster br;

    // Code when at least one marker is detected
    if(ids.size() > 1)
    {
        cerr << "Marker detektiert" << endl;
        cerr << objPoints << endl;
        cerr << imgPoints << endl;
        // Estimate position of marker in cameraCoordinateSystem
        aruco::estimatePoseSingleMarkers(corners, MarkerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
        cerr << "1" << endl;
        // Draw image with detected Marker
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cerr << "2" << endl;
        for(unsigned int i = 0; i < ids.size(); i++)
        {
            aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    MarkerSize * 0.5f);

        }
        cerr << "3" << endl;

        

        // Calculate rotation-vector of first detected marker in degree for saving
        cv::Mat R_ePSM;
        Mat tvec_ePSM;
        cv::Rodrigues(rvecs[0], R_ePSM); // R is 3x3
        tvec_ePSM.push_back(tvecs[0][0]);
        tvec_ePSM.push_back(tvecs[0][1]);
        tvec_ePSM.push_back(tvecs[0][2]);
        Mat projMatrix_ePSM;
        cv::hconcat(R_ePSM, tvec_ePSM, projMatrix_ePSM);
        Mat tvec_new_ePSM, x_rot_ePSM, y_rot_ePSM, z_rot_ePSM, eulerAngles_ePSM, R_ePSM_new;
        Mat cameraMatrix_ePSM;
        cv::decomposeProjectionMatrix(projMatrix_ePSM, cameraMatrix_ePSM, R_ePSM_new, tvec_new_ePSM, x_rot_ePSM, y_rot_ePSM, z_rot_ePSM, eulerAngles_ePSM);
        double ry_1_ePSM = -asin(R_ePSM.at<double>(2,0));
        double ry_2_ePSM = M_PI-ry_1_ePSM;
        double rx_1_ePSM = atan2((R_ePSM.at<double>(2,1)/cos(ry_1_ePSM)),(R_ePSM.at<double>(2,2)/cos(ry_1_ePSM)));
        double rx_2_ePSM = atan2((R_ePSM.at<double>(2,1)/cos(ry_2_ePSM)),(R_ePSM.at<double>(2,2)/cos(ry_2_ePSM)));
        double rz_1_ePSM = atan2((R_ePSM.at<double>(1,0)/cos(ry_1_ePSM)),(R_ePSM.at<double>(0,0)/cos(ry_1_ePSM)));
        double rz_2_ePSM = atan2((R_ePSM.at<double>(1,0)/cos(ry_2_ePSM)),(R_ePSM.at<double>(0,0)/cos(ry_2_ePSM)));
        double rx_ePSM = rx_1_ePSM*(180/M_PI);
        double ry_ePSM = ry_1_ePSM*(180/M_PI);
        double rz_ePSM = rz_1_ePSM*(180/M_PI);
        cerr << "4" << endl;
        //Calculate the euclidean distance between camera- and marker-coordinate-system for first detected marker
        float d_ePSM = sqrt((tvecs[0][0]-0)*(tvecs[0][0]-0) + (tvecs[0][1]-0)*(tvecs[0][1]-0) + (tvecs[0][2]-0)*(tvecs[0][2]-0));
        cerr << "5" << endl;

        // Calculate rotation-vector of second detected marker in degree for saving
        cv::Mat R_ePSM_02;
        cv::Rodrigues(rvecs[0], R_ePSM_02); // R is 3x3
        double ry_1_ePSM_02 = -asin(R_ePSM_02.at<double>(2,0));
        double ry_2_ePSM_02 = M_PI-ry_1_ePSM_02;
        double rx_1_ePSM_02 = atan2((R_ePSM_02.at<double>(2,1)/cos(ry_1_ePSM_02)),(R_ePSM_02.at<double>(2,2)/cos(ry_1_ePSM_02)));
        double rx_2_ePSM_02 = atan2((R_ePSM_02.at<double>(2,1)/cos(ry_2_ePSM_02)),(R_ePSM_02.at<double>(2,2)/cos(ry_2_ePSM_02)));
        double rz_1_ePSM_02 = atan2((R_ePSM_02.at<double>(1,0)/cos(ry_1_ePSM_02)),(R_ePSM_02.at<double>(0,0)/cos(ry_1_ePSM_02)));
        double rz_2_ePSM_02 = atan2((R_ePSM_02.at<double>(1,0)/cos(ry_2_ePSM_02)),(R_ePSM_02.at<double>(0,0)/cos(ry_2_ePSM_02)));
        double rx_ePSM_02 = rx_1_ePSM_02*(180/M_PI);
        double ry_ePSM_02 = ry_1_ePSM_02*(180/M_PI);
        double rz_ePSM_02 = rz_1_ePSM_02*(180/M_PI);
        cerr << "6" << endl;
        //Calculate the euclidean distance between camera- and marker-coordinate-system for second detected marker
        float d_ePSM_02 = sqrt((tvecs[0][0]-0)*(tvecs[0][0]-0) + (tvecs[0][1]-0)*(tvecs[0][1]-0) + (tvecs[0][2]-0)*(tvecs[0][2]-0));
        cerr << "7" << endl;

        // Get Object- and ImagePoints
        getObjectAndImagePoints(corners, ids, objPoints, imgPoints);
        cerr << "8" << endl;
        
        
        cerr<< objPoints << endl;
        cerr<< imgPoints << endl;
        cerr<< cameraMatrix << endl;
        cerr<< distCoeffs << endl;
        // Get position and orientation of world-coordinate-system in the camera-coordinate-system
        if (InnerCorners==0)
        {
			solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs,rvec, tvec, false, 0);
            cerr << "8.5" << endl;
        } 


        else
        {
            // If InnerCorners=1 then get Image- and ObjectPoints of the innerCorners
            inner_corners inner_corners(objPoints, imgPoints, image, ids, dictionary);
            imgPoints = inner_corners.get_imgPoints();
            objPoints = inner_corners.get_objPoints();
            solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs,rvec, tvec, false, 0);
        }
        cerr << "9" << endl;


        // Calculate rotation-vector after SolvePnP in degree for saving
        cv::Mat R_PnP;
        Mat tvec_PnP;
        cv::Rodrigues(rvec, R_PnP); // R is 3x3
        tvec_PnP.push_back(tvec[0]);
        tvec_PnP.push_back(tvec[1]);
        tvec_PnP.push_back(tvec[2]);
        Mat projMatrix_PnP;
        cv::hconcat(R_PnP, tvec_PnP, projMatrix_PnP);
        Mat tvec_new_PnP, x_rot_PnP, y_rot_PnP, z_rot_PnP, eulerAngles_PnP,R_PnP_new;
        Mat cameraMatrix_PnP;
        cv::decomposeProjectionMatrix(projMatrix_PnP, cameraMatrix_PnP, R_PnP_new, tvec_new_PnP, x_rot_PnP, y_rot_PnP, z_rot_PnP, eulerAngles_PnP);
        double ry_1_PnP = -asin(R_PnP.at<double>(2,0));
        double ry_2_PnP = M_PI-ry_1_PnP;
        double rx_1_PnP = atan2((R_PnP.at<double>(2,1)/cos(ry_1_PnP)),(R_PnP.at<double>(2,2)/cos(ry_1_PnP)));
        double rx_2_PnP = atan2((R_PnP.at<double>(2,1)/cos(ry_2_PnP)),(R_PnP.at<double>(2,2)/cos(ry_2_PnP)));
        double rz_1_PnP = atan2((R_PnP.at<double>(1,0)/cos(ry_1_PnP)),(R_PnP.at<double>(0,0)/cos(ry_1_PnP)));
        double rz_2_PnP = atan2((R_PnP.at<double>(1,0)/cos(ry_2_PnP)),(R_PnP.at<double>(0,0)/cos(ry_2_PnP)));
        double rx_pnp = rx_1_PnP*(180/M_PI);
        double ry_pnp = ry_1_PnP*(180/M_PI);
        double rz_pnp = rz_1_PnP*(180/M_PI);
        cerr << "10" << endl;

        //Calculate the euclidean distance between camera- and world-coordinate-system
        float d_pnp = sqrt((tvec[0]-0)*(tvec[0]-0) + (tvec[1]-0)*(tvec[1]-0) + (tvec[2]-0)*(tvec[2]-0));
        cerr << "11" << endl;




        //Time, rvec und tvec in struct
        PosePnPResult currpositon;
        currpositon.time = ros::Time::now();
        currpositon.vt = tvec;
        currpositon.rt = rvec;
        cerr << "12" << endl;

        //Normalizaiton of rotation vector
        std::vector<double> rv = {currpositon.rt(0),currpositon.rt(1),currpositon.rt(2)};
        double norm = sqrt(pow(rv[0],2)+pow(rv[1],2)+pow(rv[2],2));
        for (int i = 0; i < rv.size(); ++i)
            {
                rv[i] = rv[i]/norm;
            }

        tf2::Vector3 tf2rv =tf2::Vector3(rv[0], rv[1], rv[2]);

        //construct quaternion using the rotation-vector and norm
        tf2::Quaternion Q(tf2rv, norm);

        //Vecicle-Pose
        tf2::Vector3 v_marker_pose_in_cam = {currpositon.vt(0),currpositon.vt(1),currpositon.vt(2)};
        tf2::Quaternion q_marker_pose_in_cam = Q;




        
        //Vector with world coordinate pose in camera coordinate pose
        tf2::Transform trans_cam_world(q_marker_pose_in_cam, v_marker_pose_in_cam);
        //Transform back to world coordinate system
        tf2::Transform trans_world_cam = trans_cam_world.inverse();
        tf2::Vector3 trans_world_cam_trans = trans_world_cam.getOrigin();
        tf2::Quaternion trans_world_cam_rot = trans_world_cam.getRotation();

        // Calculate rotation-vector after transformation in degree for saving
        tf2::Matrix3x3 m(trans_world_cam_rot);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double rx_transformed = roll*(180/M_PI);
        double ry_transformed = pitch*(180/M_PI);
        double rz_transformed = yaw*(180/M_PI);
        //Calculate the euclidean distance between world- and camera-coordinate-system
        float d_transformed = sqrt((trans_world_cam_trans.getX()-0)*(trans_world_cam_trans.getX()-0) + (trans_world_cam_trans.getY()-0)*(trans_world_cam_trans.getY()-0) + (trans_world_cam_trans.getZ()-0)*(trans_world_cam_trans.getZ()-0));
        cerr << "13" << endl;



        //Convert tf2 data to geometry_msgs data
        geometry_msgs::Transform msgs_trans_world_cam = tf2::toMsg(trans_world_cam); 
        //@ send transformation of camera to tf2 (for visualization)
        geometry_msgs::TransformStamped transformStampedCam;
        transformStampedCam.header.stamp = ros::Time::now();
        transformStampedCam.header.frame_id = "world";
        transformStampedCam.child_frame_id = "camera";
        transformStampedCam.transform = msgs_trans_world_cam;
        br.sendTransform(transformStampedCam);

        //@ send transformation of vehicle to tf2 (for visualization)
        tf2::Quaternion Qvc;
        tf2::Vector3 Vvc =tf2::Vector3(Transform_vc[0], Transform_vc[1], Transform_vc[2]);
        Qvc.setRPY(Transform_vc[3], Transform_vc[4], Transform_vc[5]);
        tf2::Transform trans_vehicle_cam(Qvc, Vvc);
        tf2::Transform trans_world_vehicle = trans_world_cam * trans_vehicle_cam.inverse();
        tf2::Vector3 v_vehicle_pose_in_world = trans_world_vehicle.getOrigin();
        tf2::Quaternion q_vehicle_pose_in_world = trans_world_vehicle.getRotation();
        geometry_msgs::Transform msgs_trans_world_vehicle = tf2::toMsg(trans_world_vehicle);
        VehiclePoseSt.header.seq = 200;
        VehiclePoseSt.header.stamp = ros::Time::now();
        VehiclePoseSt.header.frame_id = "world";
        VehiclePoseSt.pose.position.x = v_vehicle_pose_in_world.getX();
        VehiclePoseSt.pose.position.y = v_vehicle_pose_in_world.getY();
        VehiclePoseSt.pose.position.z = v_vehicle_pose_in_world.getZ();
        VehiclePoseSt.pose.orientation.x = q_vehicle_pose_in_world.x();
        VehiclePoseSt.pose.orientation.y = q_vehicle_pose_in_world.y();
        VehiclePoseSt.pose.orientation.z = q_vehicle_pose_in_world.z();

        pose_pub_.publish(VehiclePoseSt);
        geometry_msgs::TransformStamped transformStampedVeh;
        transformStampedVeh.header.stamp = ros::Time::now();
        transformStampedVeh.header.frame_id = "world";
        transformStampedVeh.child_frame_id = "vehicle";
        transformStampedVeh.transform = msgs_trans_world_vehicle;
        br.sendTransform(transformStampedVeh);
        cerr << "14" << endl;


        if (InnerCorners==1)
        {
            //Save corner_data to txt-file
            std::string cornerfile_name = "/home/hkh/kehui/source_code_02/data_new/" + save_path + "corners.txt";
            fstream cornerfile;
            cornerfile.open(cornerfile_name, fstream::in | fstream::out | fstream::app);
            //If file does not exist. Creating new file.;
            if (!cornerfile)
            {
                cornerfile.open(cornerfile_name, fstream::in | fstream::out | fstream::trunc);
                cornerfile << imgPoints.at<float>(0,0) << " ";
                cornerfile << imgPoints.at<float>(0,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(1,0) << " ";
                cornerfile << imgPoints.at<float>(1,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(2,0) << " ";
                cornerfile << imgPoints.at<float>(2,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(3,0) << " ";
                cornerfile << imgPoints.at<float>(3,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                
                
                //Hier auskommentieren, wenn nur ein Marker
                cornerfile << imgPoints.at<float>(4,0) << " ";
                cornerfile << imgPoints.at<float>(4,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(5,0) << " ";
                cornerfile << imgPoints.at<float>(5,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(6,0) << " ";
                cornerfile << imgPoints.at<float>(6,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(7,0) << " ";
                cornerfile << imgPoints.at<float>(7,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                
                
                cornerfile.close();
                
            }
            else
            {
                cornerfile << imgPoints.at<float>(0,0) << " ";
                cornerfile << imgPoints.at<float>(0,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(1,0) << " ";
                cornerfile << imgPoints.at<float>(1,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(2,0) << " ";
                cornerfile << imgPoints.at<float>(2,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(3,0) << " ";
                cornerfile << imgPoints.at<float>(3,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                
                
                //Hier auskommentieren, wenn nur ein Marker
                cornerfile << imgPoints.at<float>(4,0) << " ";
                cornerfile << imgPoints.at<float>(4,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(5,0) << " ";
                cornerfile << imgPoints.at<float>(5,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(6,0) << " ";
                cornerfile << imgPoints.at<float>(6,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                cornerfile << imgPoints.at<float>(7,0) << " ";
                cornerfile << imgPoints.at<float>(7,1) << " ";
                cornerfile << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                

                cornerfile.close();
            }
        }

        else{
            // Save for a lot of corners
            //Save corner_data to txt-file
            std::string cornerfile_name_inner = "/home/hkh/kehui/source_code_02/data_new/" + save_path + "corners.txt";
            fstream cornerfile_inner;
            cornerfile_inner.open(cornerfile_name_inner, fstream::in | fstream::out | fstream::app);
            //If file does not exist. Creating new file.;
            if (!cornerfile_inner)
            {
                cornerfile_inner.open(cornerfile_name_inner, fstream::in | fstream::out | fstream::trunc);
                for(int i=0;i<47;i++)
                {
                    cornerfile_inner << imgPoints.at<float>(i, 0) << " ";
                    cornerfile_inner << imgPoints.at<float>(i, 1) << " ";
                    cornerfile_inner << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                }
                cornerfile_inner.close();
                
            }
            else
            {
                for(int i=0;i<47;i++)
                {
                    cornerfile_inner << imgPoints.at<float>(i, 0) << " ";
                    cornerfile_inner << imgPoints.at<float>(i, 1) << " ";
                    cornerfile_inner << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
                }
                cornerfile_inner.close();

            }
        }
        cerr << "15" << endl;
        // Save image in case corners are detected
        cv::imwrite("/home/hkh/kehui/source_code_02/data_new/" + save_path + duration + ".jpg", image);

        
        
        
        //Save solvePnP-data to txt-file
        std::string tvec_PnP_name = "/home/hkh/kehui/source_code_02/data_new/" + save_path + "tvec_solvePnP.txt";
        fstream tvec_PnP_f;
        tvec_PnP_f.open(tvec_PnP_name, fstream::in | fstream::out | fstream::app);
        //If file does not exist. Creating new file.;
        if (!tvec_PnP_f)
        {
            tvec_PnP_f.open(tvec_PnP_name, fstream::in | fstream::out | fstream::trunc);
            tvec_PnP_f << "x y z time" << std::endl;
            tvec_PnP_f << tvec[0] << " ";
            tvec_PnP_f << tvec[1] << " ";
            tvec_PnP_f << tvec[2] << " ";
            tvec_PnP_f << rx_pnp << " ";
            tvec_PnP_f << ry_pnp << " ";
            tvec_PnP_f << rz_pnp << " ";
            tvec_PnP_f << d_pnp << " ";
            tvec_PnP_f << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
            tvec_PnP_f.close();
        
        }
        else
        {
            tvec_PnP_f << tvec[0] << " ";
            tvec_PnP_f << tvec[1] << " ";
            tvec_PnP_f << tvec[2] << " ";
            tvec_PnP_f << rx_pnp << " ";
            tvec_PnP_f << ry_pnp << " ";
            tvec_PnP_f << rz_pnp << " ";
            tvec_PnP_f << d_pnp << " ";
            tvec_PnP_f << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
            tvec_PnP_f.close();

        }
        cerr << "16" << endl;

        
        //Save transformed-data to txt-file
        std::string tvec_transformed_name = "/home/hkh/kehui/source_code_02/data_new/" + save_path + "tvec_transformed.txt";
        fstream tvec_transformed;
        tvec_transformed.open(tvec_transformed_name, fstream::in | fstream::out | fstream::app);
        //If file does not exist. Creating new file.;
        if (!tvec_transformed)
        {
            tvec_transformed.open(tvec_transformed_name, fstream::in | fstream::out | fstream::trunc);
            tvec_transformed << "x y z time" << std::endl;
            tvec_transformed << transformStampedCam.transform.translation.x << " ";
            tvec_transformed << transformStampedCam.transform.translation.y << " ";
            tvec_transformed << transformStampedCam.transform.translation.z << " ";
            tvec_transformed << rx_transformed << " ";
            tvec_transformed << ry_transformed << " ";
            tvec_transformed << rz_transformed << " ";
            tvec_transformed << d_transformed << " ";
            tvec_transformed << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
            tvec_transformed.close();
        
        }
        else
        {

            tvec_transformed << transformStampedCam.transform.translation.x << " ";
            tvec_transformed << transformStampedCam.transform.translation.y << " ";
            tvec_transformed << transformStampedCam.transform.translation.z << " ";
            tvec_transformed << rx_transformed << " ";
            tvec_transformed << ry_transformed << " ";
            tvec_transformed << rz_transformed << " ";
            tvec_transformed << d_transformed << " ";
            tvec_transformed << std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count() << '\n';
            tvec_transformed.close();

        }
        cerr << "17" << endl;
        


    }
    
    // fill frame if there is an inputVideo
    inputVideo >> frame;

    // Publish image for visualisation
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
    img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg();
    image_pub_.publish(img);
    cv::waitKey(1); 
    }

    
    // Publish markerCube for visualisation
    marker_visual.header.frame_id = "world";
    marker_visual.header.stamp = ros::Time::now();
    marker_visual.ns = "basic_shapes";
    marker_visual.id = 0;
    marker_visual.type = shape;
    marker_visual.action = visualization_msgs::Marker::ADD;
    marker_visual.pose.position.x = 0;
    marker_visual.pose.position.y = 0;
    marker_visual.pose.position.z = 0.44;
    marker_visual.pose.orientation.x = 0;
    marker_visual.pose.orientation.y = 0;
    marker_visual.pose.orientation.z = 0;
    marker_visual.pose.orientation.w = 0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_visual.scale.x = 0.01;
    marker_visual.scale.y = 0.141;
    marker_visual.scale.z = 0.141;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_visual.color.r = 0.0f;
    marker_visual.color.g = 1.0f;
    marker_visual.color.b = 0.0f;
    marker_visual.color.a = 1.0;

    marker_visual.lifetime = ros::Duration();
    while (marker_pub_.getNumSubscribers() < 1)
    {
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
    }
    marker_pub_.publish(marker_visual);
    }

}

        


int main(int argc, char** argv)
{
    //Initiate ROS
    ros::init(argc, argv, "aruco_localization");
    //Create an object of class SubscribeAndPublish that will take care of everything
    aruco_localization arg;

    ros::spin();

    return 0;
    
}