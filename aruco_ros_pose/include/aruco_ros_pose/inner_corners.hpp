#ifndef INNER_CORNERS_HPP__
#define INNER_CORNERS_HPP__



#include <opencv2/calib3d.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>


#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <string>
#include "aruco_ros_pose/markerpose.hpp"
#include <boost/thread.hpp> 
#include <boost/thread/thread.hpp>


#include <fstream>
#include <iostream>

using std::vector;
using namespace cv;
#include <chrono>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
    
    

class inner_corners
{
private:
	Mat objPoints, imgPoints;
    Mat objPoints_new, imgPoints_new;
    Mat objPoints_inner_l, imgPoints_inner_l;
    Mat objPoints_inner_r, imgPoints_inner_r;
    TermCriteria TERM_CRIT = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 40, 0.001);
    Markerpose marker;

    //TODO:
    //Set correct BitSize (Nummber of ArUco-Field in one row including the two black fields left and right)
    int BitSize = 9;
    //Set correct Marker_Rotation in degree -- (also in the "markerlist.yaml" but in radiant -> example -20° rotation -> change rx=1.5708 to rx=1.2217)
    int marker_rotation = 40;


public:
	inner_corners(const cv::Mat &objP, const cv::Mat &imgP, const cv::Mat &image, std::vector<int> & ids, cv::Ptr<cv::aruco::Dictionary> &dictionary);
    vector<Point2f> shiTomasiCornerDetector(Mat image_shi, int distance);
    Mat get_marker_array(int id, cv::Ptr<cv::aruco::Dictionary> dictionary);
    float get_marker_size(int id);
    auto calc_inner_img_and_objPoints(int id, Mat imgPoints_oneMarker, Mat objPoints_oneMarker, cv::Ptr<cv::aruco::Dictionary> dictionary, Mat image_default);
    Mat get_objPoints();
    Mat get_imgPoints();

};


vector<Point2f> inner_corners::shiTomasiCornerDetector(Mat image_shi, int distance)
{
    Mat gray;
    vector<Point2f> corners_shi;

    // Original image to grayscale image
    cvtColor(image_shi, gray, cv::COLOR_BGRA2GRAY);

    // Execute the ShiTomasiCornerDetector
    // TODO: Adjust max number of corners you want to detect (in this case 50)
    goodFeaturesToTrack(gray, corners_shi,50,0.1,distance,Mat(),3,false,0.04);
    for (size_t i=0; i < corners_shi.size(); i++){
                circle(image_shi,corners_shi[i],1,Scalar(0,0,255),1,8,0);
    }

    // Get the estimated corner in subpix
    cornerSubPix(gray, corners_shi, Size(5, 5), Size(-1, -1), TERM_CRIT);

    return corners_shi;

}

// Function to get the MarkerArray ("look of the marker" -> black and white fields)
Mat inner_corners::get_marker_array(int id, cv::Ptr<cv::aruco::Dictionary> dictionary)
{

    cv::Mat markerImage;
    cv::aruco::drawMarker(dictionary, id, (BitSize*10), markerImage, 1);
    Mat line;
    Mat marker_array;
    char a[BitSize][BitSize];

    int k = 0;
    for (int i = 0; i < markerImage.rows; i+= 10)
    {   
        line = markerImage.row(i);
        cv::Size s = line.size();
        int cols = s.width;
        int j = 0;
        for (int ax = 0; ax < cols; ax += 10)
        {
    
            a[k][j] = line.at<uchar>(0,ax);
            j++;
        }
        k++;

    }
    cv::Mat markerBorder = cv::Mat(BitSize, BitSize, CV_8U, &a);
    return markerBorder;
}


// Function to get the MarkerSize (could be deletet if every marker in use-case has the same size)
float inner_corners::get_marker_size(int id)
{
    
    float MarkerSize;
    MarkerSize = marker.getMarkersize(id);
    return MarkerSize;
}


// Return function for main-script
Mat inner_corners::get_imgPoints()
{
	return imgPoints_inner_l;
}

Mat inner_corners::get_objPoints()
{
	return objPoints_inner_l;
}

struct points {Mat img; Mat obj;};

// Function with ShiTomasiCornerDetector to get the inner coners
auto inner_corners::calc_inner_img_and_objPoints(int id, Mat imgPoints_oneMarker, Mat objPoints_oneMarker, cv::Ptr<cv::aruco::Dictionary> dictionary, Mat image_default)
{
    // Defining the ROI of marker
    Mat image_zoomed;
    float x1 = imgPoints_oneMarker.at<float>(0,0);
    float y1 = imgPoints_oneMarker.at<float>(0,1);    
    float x2 = imgPoints_oneMarker.at<float>(1,0);
    float y2 = imgPoints_oneMarker.at<float>(1,1);
    float edge = (x2 - x1)/15;
    float length = x2 - x1;
    float length_total = length+2*edge;
    cv::Rect corner_ROI(x1-edge,y1-edge,length+2*edge,length+2*edge);
    image_zoomed = image_default(corner_ROI);
    vector<Point2f> corners_shi;

    // Get corners
    corners_shi = shiTomasiCornerDetector(image_zoomed, 5);

    //Corners of marker to Mat-object results in ImagePoints
    Mat imgPoints_inner;
    for(int i=0; i<corners_shi.size(); i++)
    {   
        imgPoints_inner.push_back(cv::Point2f((x1 - edge + corners_shi[i].x), y1 - edge + corners_shi[i].y));

    }

    // Get infos of marker
    Mat markerBorder = get_marker_array(id, dictionary);
    float MarkerSize = get_marker_size(id);

    //Get position of inner corners in world-coordinate-system (ObjectPoints) of marker
    int nBitsSquared = BitSize-2;
    float bitSize =  450/BitSize;
    std::vector<cv::Point3f> innerCorners;
    std::vector<cv::Point2f> inners;
    
    for(int y=0; y< markerBorder.rows-1; y++)
    {
        for(int x=0; x< markerBorder.cols-1; x++)
        {

            if(     ((markerBorder.at<uchar>(y, x) == markerBorder.at<uchar>(y+1, x+1)) &&
                    (markerBorder.at<uchar>(y, x) != markerBorder.at<uchar>(y, x+1) ||
                    markerBorder.at<uchar>(y, x) != markerBorder.at<uchar>(y+1, x)))

                    ||

                    ((markerBorder.at<uchar>(y, x+1) == markerBorder.at<uchar>(y+1, x)) &&
                    (markerBorder.at<uchar>(y, x+1) != markerBorder.at<uchar>(y, x) ||
                    markerBorder.at<uchar>(y, x+1) != markerBorder.at<uchar>(y+1, x+1)))
                )

                {
                    innerCorners.push_back(cv::Point3f(0, (x-nBitsSquared/2.f), -(y-nBitsSquared/2.f)) * bitSize);
                    inners.push_back(cv::Point2f((x-nBitsSquared/2.f), -(y-nBitsSquared/2.f)) * bitSize);

                }

        }
    }

    vector<Point3f> d_corners_in_m;
    for(int k=0; k<inners.size(); k++)
    {
        d_corners_in_m.push_back(Point3f(0,MarkerSize/2,MarkerSize/2));
    }


    Mat innerCorners_mat;
    Mat(innerCorners).copyTo(innerCorners_mat);
    Mat d_corners_in_m_mat;
    Mat(d_corners_in_m).copyTo(d_corners_in_m_mat);
    d_corners_in_m_mat=d_corners_in_m_mat/225;
    Mat objPoints_final;
    objPoints_final = d_corners_in_m_mat.mul(innerCorners_mat);
    objPoints_final.push_back(Point3f(0, -MarkerSize / 2.f, (MarkerSize / 2.f)));
    objPoints_final.push_back(Point3f(0, MarkerSize / 2.f, (MarkerSize / 2.f)));
    objPoints_final.push_back(Point3f(0, MarkerSize / 2.f, (-MarkerSize / 2.f)));
    objPoints_final.push_back(Point3f(0, -MarkerSize / 2.f, (-MarkerSize / 2.f)));

    // TODO:
    // In my setup the markers are on the y-axis faced to x-axis in the world-coordinate-system (wks)
    // in reality the markers might be on the x-axis faced to the y-axes in the wks
    // it's possible to get the rotation out of the "markerlist.yaml" but this is work for the future (probably for you :D)
    std::vector<cv::Point3f> xyzmarkerworld = marker.getmarkerworldcoordinates(id);
    Mat xyzmarkerworld_mat;
    Mat(xyzmarkerworld).copyTo(xyzmarkerworld_mat);

    for(int i=0; i < d_corners_in_m.size()+4; i++)
    {   
        objPoints_final.at<float>(i, 0) = -sin(-marker_rotation*M_PI/180)*objPoints_final.at<float>(i,2) + xyzmarkerworld_mat.at<float>(0,0);    //objPoints_final.at<float>(i, 0)+xyzmarkerworld_mat.at<float>(0,0);
        objPoints_final.at<float>(i, 1) = objPoints_final.at<float>(i, 1)+xyzmarkerworld_mat.at<float>(0,1);
        objPoints_final.at<float>(i, 2) =  (objPoints_final.at<float>(i, 2)*cos(-marker_rotation*M_PI/180))+xyzmarkerworld_mat.at<float>(0,2);  //objPoints_final.at<float>(i, 2)+xyzmarkerworld_mat.at<float>(0,2);
    }


    // Get ImagePoints of inner corners with homographie-matrix
    std::vector<cv::Point2f> points3d;
    points3d.push_back(Point2f(-225,225));
    points3d.push_back(Point2f(225,225));
    points3d.push_back(Point2f(225,-225));
    points3d.push_back(Point2f(-225,-225));
    cv::Mat H = cv::findHomography(points3d, imgPoints_oneMarker);
    inners.push_back(Point2f(-225,225));
    inners.push_back(Point2f(225,225));
    inners.push_back(Point2f(225,-225));
    inners.push_back(Point2f(-225,-225));
    std::vector<cv::Point2f> img_points_final;
    cv::perspectiveTransform(inners, img_points_final, H);
    Mat imgpoints_final;
    Mat(img_points_final).copyTo(imgpoints_final);

    cv::Size s = imgPoints_inner.size();
    int rows = s.height;

    Mat imgPoints_inner_final, objPoints_inner_final;

    // Check if detected corners are close to ImagePoints created with homographie-matrix to validate if the pixel is a real InnerCorner
    for(int i=0; i < img_points_final.size(); i++)
    {
        cv::Rect rect(imgpoints_final.at<float>(i,0)-5, imgpoints_final.at<float>(i,1)-5, 10,10);
        
        for(int j=0; j < rows; j++)
        {
        cv::Point2f p_0(imgPoints_inner.at<float>(j,0), imgPoints_inner.at<float>(j,1)); 
        if (rect.contains(p_0))
        {
            imgPoints_inner_final.push_back(p_0);

            objPoints_inner_final.push_back(Point3f(objPoints_final.at<float>(i,0), objPoints_final.at<float>(i,1), objPoints_final.at<float>(i,2)));

        }
        }
    }

    points Points;
    Points.img = imgPoints_inner_final;
    Points.obj = objPoints_inner_final;
    return Points;

}



// Function to get the Image- and ObjectPoints of the inner corners
inner_corners::inner_corners(const cv::Mat &objP, const cv::Mat &imgP, const cv::Mat &image, std::vector<int> & ids, cv::Ptr<cv::aruco::Dictionary> &dictionary)
{
    objPoints = objP;
	imgPoints = imgP;

    Mat image_default = image;

    // Variables with single ids
    int id1_old = ids[0];
    int id2_old = ids[1];
    int id1, id2;


    // Sort image- and objectPoints, so the second half is the right marker
    if(objPoints.at<float>(3,1) < objPoints.at<float>(4,1))
    {
        
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(0,0),imgPoints.at<float>(0,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(1,0),imgPoints.at<float>(1,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(2,0),imgPoints.at<float>(2,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(3,0),imgPoints.at<float>(3,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(4,0),imgPoints.at<float>(4,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(5,0),imgPoints.at<float>(5,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(6,0),imgPoints.at<float>(6,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(7,0),imgPoints.at<float>(7,1)));

        objPoints_new.push_back(Point3f(objPoints.at<float>(0,0),objPoints.at<float>(0,1),objPoints.at<float>(0,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(1,0),objPoints.at<float>(1,1),objPoints.at<float>(1,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(2,0),objPoints.at<float>(2,1),objPoints.at<float>(2,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(3,0),objPoints.at<float>(3,1),objPoints.at<float>(3,2)));      
        objPoints_new.push_back(Point3f(objPoints.at<float>(4,0),objPoints.at<float>(4,1),objPoints.at<float>(4,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(5,0),objPoints.at<float>(5,1),objPoints.at<float>(5,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(6,0),objPoints.at<float>(6,1),objPoints.at<float>(6,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(7,0),objPoints.at<float>(7,1),objPoints.at<float>(7,2)));
        id1 = id1_old;
        id2 = id2_old; 
        

        
    }
    else
    {
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(4,0),imgPoints.at<float>(4,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(5,0),imgPoints.at<float>(5,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(6,0),imgPoints.at<float>(6,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(7,0),imgPoints.at<float>(7,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(0,0),imgPoints.at<float>(0,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(1,0),imgPoints.at<float>(1,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(2,0),imgPoints.at<float>(2,1)));
        imgPoints_new.push_back(Point2f(imgPoints.at<float>(3,0),imgPoints.at<float>(3,1)));

        objPoints_new.push_back(Point3f(objPoints.at<float>(4,0),objPoints.at<float>(4,1),objPoints.at<float>(4,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(5,0),objPoints.at<float>(5,1),objPoints.at<float>(5,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(6,0),objPoints.at<float>(6,1),objPoints.at<float>(6,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(7,0),objPoints.at<float>(7,1),objPoints.at<float>(7,2)));    
        objPoints_new.push_back(Point3f(objPoints.at<float>(0,0),objPoints.at<float>(0,1),objPoints.at<float>(0,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(1,0),objPoints.at<float>(1,1),objPoints.at<float>(1,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(2,0),objPoints.at<float>(2,1),objPoints.at<float>(2,2)));
        objPoints_new.push_back(Point3f(objPoints.at<float>(3,0),objPoints.at<float>(3,1),objPoints.at<float>(3,2)));
        id1 = id2_old;
        id2 = id1_old; 
        
    }
   

    // Create arrays with Image- and ObjecPoints for seperate marker
    cv::Size imgPointsSize = imgPoints_new.size();
    int imgPointsSize_rows = imgPointsSize.height;
    Mat imgPoints_l, objPoints_l;
    Mat imgPoints_r, objPoints_r;

    for(int i=0; i<imgPointsSize_rows-4; i++)
    {
        imgPoints_l.push_back(Point2f(imgPoints_new.at<float>(i,0), imgPoints_new.at<float>(i,1)));
        objPoints_l.push_back(Point3f(objPoints_new.at<float>(i,0),objPoints_new.at<float>(i,1),objPoints_new.at<float>(i,2)));
            
    }

    for(int i=4; i<imgPointsSize_rows; i++)
    {
        imgPoints_r.push_back(Point2f(imgPoints_new.at<float>(i,0), imgPoints_new.at<float>(i,1)));
        objPoints_r.push_back(Point3f(objPoints_new.at<float>(i,0),objPoints_new.at<float>(i,1),objPoints_new.at<float>(i,2)));
            
    }

    points Points_left;
    points Points_right;

    Points_left = calc_inner_img_and_objPoints(id1, imgPoints_l, objPoints_l, dictionary, image_default);
    Points_right = calc_inner_img_and_objPoints(id2, imgPoints_r, objPoints_r, dictionary, image_default);
    objPoints_inner_l = Points_left.obj;
    imgPoints_inner_l = Points_left.img;
    objPoints_inner_r = Points_right.obj;
    imgPoints_inner_r = Points_right.img;

    // Connect the inner Image- and ObjectPoints from left and right marker
    objPoints_inner_l.push_back(objPoints_inner_r);
    imgPoints_inner_l.push_back(imgPoints_inner_r);

    }



#endif