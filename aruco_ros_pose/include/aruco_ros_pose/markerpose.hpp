#ifndef MARKERPOSE_HPP_
#define MARKERPOSE_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace cv;
using namespace std;

class Markerpose
{
	private:
        //@ Temporarily store the pose and id of the markers
		std::vector < std::vector <double> > pose_marker_id_list;
        //@ path of markerlist.yaml
        std::string marker_list_path;
        //@ The coordinates of the four corner points of the marker in the world coordinate system
        std::vector<cv::Vec3f> objPoints;

	public:	
        //@ Correspondences of marker pose, size and id
		std::unordered_map<int, std::vector<double>> marker_pose;
        //@ Correspondences of the coordinates of the four corner points of the marker and its id
        std::unordered_map<int, std::vector<cv::Vec3f>> marker_corners;
        //@ Constructor
		Markerpose();
        //@ Construct the unordered map "marker_pose" using the "pose_marker_id_list"
		std::vector<double> FillMarker(const std::vector<double> & vv);
        //@ print pose of markers
        void MarkerposeShow();
        //@ get side length of marker given id
        double getMarkersize(int markerid);
        //@ read the "markerlist.yaml" to "pose_marker_id_list"
        void ReadMarkerfromFile(std::vector<std::vector<double> > & MsL, const std::string & MLPath);
        //@ Solve the coordinates of the four corner points of the marker in the world coordinate system according to id
        void transObjectPointsToWorld(int id);
        //@ get four corner points of the marker in the world coordinate system given id
        void getObjectPointsInWorld(int id, std::vector<cv::Vec3f> & objPs);
        //@ get the side length of the smallest marker in the map
        double getMinMarkerSize();
        std::vector<cv::Point3f> getmarkerworldcoordinates(int markerid);
};

//@ Constructor will automatically build two unordered maps using "markerlist.yaml" file:
//@ "marker_pose" and "marker_corners"
Markerpose::Markerpose()
{	
    std::string marker_list_path = ros::package::getPath("aruco_ros_pose");

	ReadMarkerfromFile(pose_marker_id_list, marker_list_path);

	for (int i = 0; i < pose_marker_id_list.size(); ++i)
	{
		marker_pose.insert(std::pair<int, std::vector<double> >(int(pose_marker_id_list[i][0]),FillMarker(pose_marker_id_list[i])));
        transObjectPointsToWorld(int(pose_marker_id_list[i][0]));
        marker_corners.insert(std::pair<int, std::vector<cv::Vec3f>>(int(pose_marker_id_list[i][0]),objPoints));
	}
}

std::vector<double> Markerpose::FillMarker(const std::vector<double> & vv)
{
	tf2::Quaternion trans;
    trans.setRPY(vv[4], vv[5], vv[6]);
    std::vector<double> qq = {vv[1], vv[2], vv[3], trans.x(), trans.y(), trans.z(), trans.w(), vv[7]};
    return qq;
}

// @@ return: the size of the smallest markersize
double Markerpose::getMinMarkerSize()
{
    std::vector<double> markersizelist;
    for (int i = 0; i < pose_marker_id_list.size(); ++i)
    {
        markersizelist.push_back(pose_marker_id_list[i][7]);
    }

    std::vector<double>::iterator smallest = std::min_element(markersizelist.begin(), markersizelist.end());

    return *smallest;
}

// @@ Update the "objPoints" object with the map data 
void Markerpose::transObjectPointsToWorld(int id)
{
    objPoints.clear();

    double markersize = getMarkersize(id);

    std::vector<Vec3f> PM4 = {
        Vec3f(-markersize / 2.f, markersize / 2.f, 0),
        Vec3f(markersize / 2.f, markersize / 2.f, 0),
        Vec3f(markersize / 2.f, -markersize / 2.f, 0),
        Vec3f(-markersize / 2.f, -markersize / 2.f, 0)
    };

    std::unordered_map <int, std::vector<double> >::iterator iter1;
    iter1 = marker_pose.find(id);
    if(iter1 != marker_pose.end())
    {   
        std::vector<double> marker_pose_temp = iter1->second;
        tf2::Vector3 t_wm = {marker_pose_temp[0],
              marker_pose_temp[1],marker_pose_temp[2]};

        tf2::Quaternion q_wm = { marker_pose_temp[3],
          marker_pose_temp[4],marker_pose_temp[5],marker_pose_temp[6]};

        tf2::Transform T_wm(q_wm, t_wm);

        for (int i = 0; i < 4; ++i)
        {
            tf2::Vector3 pm = { PM4[i][0], PM4[i][1], PM4[i][2] };
            tf2::Vector3 pw = T_wm * pm;
            objPoints.push_back(Vec3f(pw[0],pw[1],pw[2]));
        }
    }
}

std::vector<cv::Point3f> Markerpose::getmarkerworldcoordinates(int markerid)
{
    objPoints.clear();

    double markersize = getMarkersize(markerid);


    std::unordered_map <int, std::vector<double> >::iterator iter1;
    iter1 = marker_pose.find(markerid);
    std::vector<cv::Point3f> t_wm;
    if(iter1 != marker_pose.end())
    {   
        std::vector<double> marker_pose_temp = iter1->second;
        t_wm.push_back(cv::Point3f(marker_pose_temp[0], marker_pose_temp[1], marker_pose_temp[2]));
    
    }
    return t_wm;

}


void Markerpose::getObjectPointsInWorld(int id, std::vector<cv::Vec3f> & objPs)
{   
    std::unordered_map <int, std::vector<cv::Vec3f> >::iterator iterr;
    iterr = marker_corners.find(id);
    if(iterr != marker_corners.end())
    {   
        std::vector<cv::Vec3f> temp = iterr->second;
        objPs.assign(temp.begin(), temp.end());
    }
}

void Markerpose::MarkerposeShow()
{
    //@ Traverse the map
    for (int row = 0; row < pose_marker_id_list.size(); row++) {
        for (int column = 0; column < pose_marker_id_list[row].size(); column++) {
            std::cout << pose_marker_id_list[row][column] << " " ;
        }
        std::cout << std::endl; 
    }
}

double Markerpose::getMarkersize(int mid)
{
    //@ find id in map
    std::unordered_map <int, std::vector<double> >::iterator iterr;
    iterr = marker_pose.find(mid);

    //@ If a marker is found, then retrun marker size
    if(iterr != marker_pose.end())
    {
        std::vector<double> lv = iterr->second;
        return lv[7];
    }
    else
        return -1;
}

void Markerpose::ReadMarkerfromFile(std::vector < std::vector <double> > &MarkersList, const std::string & MarkerListPath)
{
    std::string line;
    std::stringstream ss;
    std::vector<double> MarkerList;

    double i;
    std::ifstream posefile("/home/schulpius/catkin_ws/src/aruco_ros_pose/config/markerlist.yaml");

    if (posefile.is_open()) {
        while (getline (posefile, line)) {
            ss.clear();
            ss.str("");
            ss.str(line);
            MarkerList.clear();

            if ((line.find("#") != std::string::npos && int(line.find("#")) < 7) ||
                line.find_first_not_of(" ") == std::string::npos ) {continue;}

            while(ss >> i) {
                MarkerList.push_back(i);

                if (ss.peek() == ',' || ss.peek() == ' '|| ss.peek() == '\t') {
                    ss.ignore();
                }

                if (ss.peek() == '#'){
                    ss.clear();
                    break;
                }

            }
            MarkersList.push_back(MarkerList);
        }
        posefile.close();
    }

    else std::cout << "Unable to open file";
}

#endif