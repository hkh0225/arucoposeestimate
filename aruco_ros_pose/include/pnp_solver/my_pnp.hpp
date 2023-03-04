//
// "__        __   _  _ _                      
// "\ \      / /__(_)(_|_) __ _ 
// " \ \ /\ / / _ \ || | |/ _` | 
// "  \ V  V /  __/ || | | (_| |  
// "   \_/\_/ \___|_|/ |_|\__,_|    
// "               |__/                                        
// "
// 
//
#ifndef MY_PNP_HPP_
#define MY_PNP_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>

using namespace cv;

class Mypnp
{
private:
	cv::Mat objPoints;
	cv::Mat imgPoints;
	cv::Mat cameraMatrix;
	Eigen::AngleAxisd rvec_eigen;
	Eigen::Vector3d tvec_eigen;
	cv::Vec3d rvec,tvec;
public:
	//构造并读取数据 Daten lesen
	Mypnp(const cv::Mat &objP, const cv::Mat &imgP, const cv::Mat &cMatrix);
	//计算位姿 Pose berechnen
	void solvepnp();
	//输出pose Pose ausgeben
	cv::Vec3d get_rvec();
	cv::Vec3d get_tvec();
	Eigen::Affine2d Find2DAffineTransform(Eigen::Matrix2Xd in, Eigen::Matrix2Xd out);
	Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out);
};

Mypnp::Mypnp(const cv::Mat &objP, const cv::Mat &imgP, const cv::Mat &cMatrix)
{
	objPoints = objP;
	imgPoints = imgP;
	cameraMatrix = cMatrix;
}

cv::Vec3d Mypnp::get_rvec()
{
	return rvec;
}

cv::Vec3d Mypnp::get_tvec()
{
	return tvec;
}


void Mypnp::solvepnp()
{
	std::vector< Point3f > obj;
    std::vector< Point2f > img;
    Mat(objPoints).copyTo(obj);
    Mat(imgPoints).copyTo(img);
    Eigen::Matrix3Xd input_mat_obj=Eigen::Matrix3Xd::Zero(3,obj.size());
    Eigen::Matrix3Xd input_mat_img=Eigen::Matrix3Xd::Zero(3,img.size());
    Eigen::Matrix2Xd input_mat_obj_2d = Eigen::Matrix2Xd::Zero(2,obj.size());
    Eigen::Matrix2Xd input_mat_img_2d = Eigen::Matrix2Xd::Zero(2,img.size());
    for (int i=0; i<obj.size();i++)
    {
        input_mat_obj(0,i) = obj[i].x;
        input_mat_obj(1,i) = obj[i].y;
        input_mat_obj(2,i) = obj[i].z;
    }
   // std::cout<<"eigen矩阵是："<<input_mat_obj<<std::endl;
    double x1,y1,z1,u1,v1,x2,y2,z2,u2,v2;
    double k1,k2;
    double g1,g2;
    double fx,cx,fy,cy;
    double H,K;
    
    fx = cameraMatrix.at<double>(0,0);
    cx = cameraMatrix.at<double>(0,2);
    fy = cameraMatrix.at<double>(1,1);
    cy = cameraMatrix.at<double>(1,2);
    
    K=(img[0].y-cy)/(img[3].y-cy);
    H=(obj[0].z-obj[3].z*K)/(1-K);

       for (int i=0; i<img.size();i++)
    {
        input_mat_img(2,i) = (H-obj[i].z)*fy/(img[i].y-cy);
        input_mat_img(1,i) = H-obj[i].z;
        input_mat_img(0,i) = (img[i].x-cx)*input_mat_img(2,i)/fx;
    }
    //std::cout<<"eigen图像矩阵是："<<input_mat_img<<std::endl;

    //给2d的输入输出赋值
    input_mat_obj_2d.row(0)=input_mat_obj.row(0);
    input_mat_obj_2d.row(1)=input_mat_obj.row(1);
    input_mat_img_2d.row(0)=input_mat_img.row(0);
    input_mat_img_2d.row(1)=input_mat_img.row(2);

    //Eigen::Affine3d A=Find3DAffineTransform(input_mat_obj,input_mat_img);
    Eigen::Affine2d B=Find2DAffineTransform(input_mat_obj_2d,input_mat_img_2d);
    Eigen::Matrix3d R=Eigen::Matrix3d::Zero(3,3);
    R(0,0)=B.linear()(0,0);
    R(0,1)=B.linear()(0,1);
    R(1,2)=-1;
    R(2,0)=B.linear()(1,0);
    R(2,1)=B.linear()(1,1);
    Eigen::Vector3d t;
    t(0)=B.translation()(0);
    t(1)=H;
    t(2)=B.translation()(1);
    //std::cout<<"mypnp_R: "<<R<<std::endl;
    //std::cout<<"t: "<<t<<std::endl;
    
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(R);
    //std::cout<<"rvec: "<<rotation_vector.axis()<<std::endl;
    //std::cout<<"theta: "<<rotation_vector.angle()<<std::endl;
    rvec_eigen = rotation_vector;
    tvec_eigen = t;
	rvec(0) = rvec_eigen.axis()(0)*rvec_eigen.angle();
	rvec(1) = rvec_eigen.axis()(1)*rvec_eigen.angle();
	rvec(2) = rvec_eigen.axis()(2)*rvec_eigen.angle();
	tvec(0) = tvec_eigen(0);
	tvec(1) = tvec_eigen(1);
	tvec(2) = tvec_eigen(2);
}

Eigen::Affine2d Mypnp::Find2DAffineTransform(Eigen::Matrix2Xd in, Eigen::Matrix2Xd out) {
  // Default output"            
  Eigen::Affine2d A;
  A.linear() = Eigen::Matrix2d::Identity(2, 2);
  A.translation() = Eigen::Vector2d::Zero();

  if (in.cols() != out.cols())
    throw "Find2DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) 
  {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;
  scale=1;
  // Find the centroids then shift to the origin
  Eigen::Vector2d in_ctr = Eigen::Vector2d::Zero();
  Eigen::Vector2d out_ctr = Eigen::Vector2d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix2d I = Eigen::Matrix2d::Identity(2, 2);
  I(1, 1) = d;
  Eigen::Matrix2d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}

Eigen::Affine3d Mypnp::Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;
  scale=1;
  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}


#endif
