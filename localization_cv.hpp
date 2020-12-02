#ifndef LOCALIZATION_CV_HPP_
#define LOCALIZATION_CV_HPP_



#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <string>
#include <fstream>

#include <boost/thread.hpp> 
#include <boost/thread/thread.hpp>

#include <Eigen/Dense>


#include "localization_with_artrack_cv/markerpose.hpp"
#include "localization_with_artrack_cv/utils.h"

using std::vector;
using namespace cv;
using Eigen::MatrixXd;

//@ struct to hold result of solution of PnP-Problem and the corresponding timestamp
struct PosePnPResult
{
    ros::Time time;
    cv::Vec3d vt;
    cv::Vec3d rt;
};

class LocalizationCv
{
private:
    //@ ros Publisher and Subscriber
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    // image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_pub_utm_;
    ros::Publisher pose_pub_cov_;
    ros::Publisher pose_pub_cov_utm_;
    //@ Subscriber: allow concurrent callbacks
    ros::Subscriber imagesub_;
    ros::SubscribeOptions concurrent_callback;

    //@ aruco dictionary and aruco detection parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

    //@ camera intrinsics
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    //@ focal length x and y
    double camerafx;
    double camerafy;
    //@ average focal length
    double cameraf;

    //@ side length of the smallest marker in the map
    double minmarkersize;

    //@ Object instance: load marker map
    Markerpose marker;

    //@ sequence ID of the previous image message
    int preSequenceId;
    //@ previous valid Pose(result of PnP-Problem) of camera
    PosePnPResult prePose;

    //@ additional detection parameters
    //@ Region of interest(ROI),set a region in the image to detect markers 
    std::vector<int> ROIvec;
    //@ Transform vehicle to camera
    std::vector<double> Transform_vc;

    //@ Refer to the readme.md file for the meaning of the following parameters
    double H_VehicleCoordinate;
    double maxDetectDistance;
    double minDetectDistance;
    bool RemoveOutlierSpeed;
    bool RemoveOutlierHight;
    double RemoveOutlierSpeedThreshold;
    double RemoveOutlierHightThreshold;
    bool LocalizationWithSingleMarker;
    double RelativeMarkerSizeToSolvePnP;
    std::vector<std::vector<double>> StdDevSingleMarker;
    std::vector<std::vector<double>> StdDevDoubleMarker;

    // std::ofstream fout;

public:
    //@ Constructor: use video topic for initialization
	LocalizationCv(std::string & imagetopic);
    //@ Destructor
	~LocalizationCv();

    //@ Determine whether the PnP solution result is an outlier based on speed
    //@@input:: pre: Previous PnP result, curr: Current PnP result, threshold: speed threshold, flag: use threshold or not
    //@@output:: is outlier-> false; is not outlier -> true
	bool IsOutlier_Speed(const PosePnPResult &pre, const PosePnPResult &curr, double threshold, bool flag);

    //@ Determine whether the PnP solution result is an outlier based on the height of the camera
    //@@input:: curr: Current PnP result, threshold: speed threshold, flag: use threshold or not
    //@@output:: is outlier-> false; is not outlier -> true
	bool IsOutlier_Hight(const PosePnPResult &curr, double threshold, bool flag);

    //@ read detector parameters from "detector_params.yaml"
	void readDetectorParameters(cv::Ptr<aruco::DetectorParameters> &params);

    //@ Generate 2d-3d correspondences as PnP input according to the detection result
    //@@ input:: detectedIds:The ids of the detected markers, detectedCorners: pixel coordinates of marker corner
    //@@ output:: objPoints:marker corner coordinates in 3D world, imgPoints: The coordinates of the marker corner in the image
	void getObjectAndImagePoints(InputArrayOfArrays detectedCorners,
    		InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints);

    //@ Return the perimeter of a marker given id
	double getMinDetectionPerimeter(int id);

    //@ Remove markers outside the specified detection range
	void removeOutofRange(std::vector<int> & ids, std::vector<std::vector<cv::Point2f> > & corners, 
			std::vector<double> & perimeters);

    //@ Convert coordinate from parking lot coordinate system to UTM coordinate system
	tf2::Transform worldToUTM(const tf2::Transform & tf);

    //@ Generate variance of x, y, yaw angle
    bool getPoseVariance(const std::vector<int> &ids, const std::vector<double> &perimeters, std::vector<double> &Var);

    //@ Node callback function
	void imageCb(const sensor_msgs::ImageConstPtr& msg);

	// void imageCbthread(const sensor_msgs::ImageConstPtr& msg);
    void mysolvepnp(const cv::Mat &objPoints, const cv::Mat &imgPoints, const cv::Mat &cameraMatrix, Eigen::AngleAxisd &rvec, Eigen::Vector3d &tvec);


    Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out);
    Eigen::Affine2d Find2DAffineTransform(Eigen::Matrix2Xd in, Eigen::Matrix2Xd out);
};


LocalizationCv::LocalizationCv(std::string & imagetopic) : it_(nh_)
{
    // image_sub_ = it_.subscribe(imagetopic, 1,&LocalizationCv::imageCb, this);
    // image_sub_ = it_.subscribe(imagetopic, 10,&LocalizationCv::imageCbthread, this);
    image_pub_ = it_.advertise("/localization_cv_image_detected_markers", 1);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("vehicle_pose", 1);
    pose_pub_utm_ = nh_.advertise<geometry_msgs::PoseStamped>("vehicle_pose_utm", 1);
    pose_pub_cov_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("vehicle_pose_cov", 1);
    pose_pub_cov_utm_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("vehicle_pose_cov_utm", 1);

    //@ allow concurrent callbacks mode
    //@ concurrent_callback.template init<sensor_msgs::Image>(imagetopic, 10, &LocalizationCv::imageCb);
    concurrent_callback.template init<sensor_msgs::Image>(imagetopic, 3, boost::bind(&LocalizationCv::imageCb, this, _1));
    concurrent_callback.transport_hints = ros::TransportHints();
    concurrent_callback.allow_concurrent_callbacks = true;
    imagesub_ = nh_.subscribe(concurrent_callback);

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    parameters = cv::aruco::DetectorParameters::create();

    //@ read camera intrinsics parameters from "camera_calibration.yaml"
    Readcameracalib(cameraMatrix, distCoeffs);

    //@ The focal length of the camera
    camerafx = cameraMatrix.at<double>(0,0);
    camerafy = cameraMatrix.at<double>(1,1);
    cameraf = (camerafx + camerafy) / 2;

    //@ get the side length of the smallest marker in the map
    minmarkersize = marker.getMinMarkerSize();

    preSequenceId = 0;

    //@ set all detection parameters from "detector_params.yaml"
    readDetectorParameters(parameters);
    
    // fout.open("/home/likun/catkin_ws/src/localization_with_artrack_cv/data/time.txt");
}

LocalizationCv::~LocalizationCv()
{   
    // fout.close();
}

bool LocalizationCv::IsOutlier_Speed(const PosePnPResult &pre, const PosePnPResult &curr, double threshold, bool flag)
{   
    if(!flag)
        return true;
    PosePnPResult temp;
    ros::Duration Dur = curr.time - pre.time;
    double time = Dur.toSec();

    double cx,cy,cz;
    cx = curr.vt(0);
    cy = curr.vt(1);
    cz = curr.vt(2);

    double px,py,pz;
    px = pre.vt(0);
    py = pre.vt(1);
    pz = pre.vt(2);

    //@ Calculate the speed of the vehicle in 3D space
    double V3Distance = sqrt(pow(cx-px,2) + pow(cy-py,2) + pow(cz-pz,2));
    double V3Speed = V3Distance / (time+0.00001);

    if (V3Speed < threshold) //threshold = speed in m/s
        return true;
    return false;
}

bool LocalizationCv::IsOutlier_Hight(const PosePnPResult &curr, double threshold, bool flag)
{   
    if (!flag)
        return true;

    if ( Transform_vc.empty() || H_VehicleCoordinate == 0)
        return true;

    double h1 = Transform_vc[2];
    //@ The height of the camera from the ground(Static installation height)
    double H = h1 + H_VehicleCoordinate;

    //@ normalazition of rotation vector
    std::vector<double> rv = {curr.rt(0),curr.rt(1),curr.rt(2)};

    double norm = sqrt(pow(rv[0],2)+pow(rv[1],2)+pow(rv[2],2));

    for (int i = 0; i < rv.size(); ++i)
    {
        rv[i] = rv[i]/norm;
    }

    tf2::Vector3 tf2rv =tf2::Vector3(rv[0], rv[1], rv[2]);

    //@ construct quaternion with rotation vector
    tf2::Quaternion Q(tf2rv, norm);

    tf2::Vector3 v_marker_pose_in_cam = { curr.vt(0), curr.vt(1), curr.vt(2)};
    tf2::Quaternion q_marker_pose_in_cam = Q;

    //@ Tcw(world coordinate pose in camera coordinate system)
    tf2::Transform trans_cam_world(q_marker_pose_in_cam, v_marker_pose_in_cam);

    //@ Twc(camera pose in world coordinate system)
    tf2::Transform trans_world_cam = trans_cam_world.inverse();

    tf2::Vector3 v_cam_pose_in_world = trans_world_cam.getOrigin();
    // tf2::Quaternion q_cam_pose_in_world = trans_world_cam.getRotation();

    //@ Camera height based on PnP results
    double z = v_cam_pose_in_world.getZ();

    double Th = threshold+H;
    double Tl = -threshold+H;

    //@ Determine if the camera height is within the threshold
    if (z < Th && z > Tl)
        return true;
    else
        return false;
}

void LocalizationCv::readDetectorParameters(cv::Ptr<aruco::DetectorParameters> &params)
{   
    std::string paras_path = ros::package::getPath("localization_with_artrack_cv");
    paras_path = paras_path + "/config/detector_params.yaml";
    cv::FileStorage fs(paras_path, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_ERROR("Invalid detector parameters file");
        return;
    }

    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["maxDetectDistance"] >> maxDetectDistance;
    fs["minDetectDistance"] >> minDetectDistance;
    fs["detectROI"] >> ROIvec;
    fs["Transform_vc"] >> Transform_vc;
    fs["H_VehicleCoordinate"] >> H_VehicleCoordinate;
    fs["RemoveOutlierSpeed"] >> RemoveOutlierSpeed;
    fs["RemoveOutlierHight"] >> RemoveOutlierHight;
    fs["RemoveOutlierSpeedThreshold"] >> RemoveOutlierSpeedThreshold;
    fs["RemoveOutlierHightThreshold"] >> RemoveOutlierHightThreshold;
    fs["LocalizationWithSingleMarker"] >> LocalizationWithSingleMarker;
    fs["RelativeMarkerSizeToSolvePnP"] >> RelativeMarkerSizeToSolvePnP;

    cv::FileNode fn = fs["StandardDeviationPoseEstimationSingleMarker"];
    cv::FileNodeIterator current = fn.begin(), it_end = fn.end();
    for (; current != it_end; ++current)
    {
        vector<double> tmp;
        FileNode item = *current;
        item >> tmp;
        StdDevSingleMarker.push_back(tmp);
    }

    fn = fs["StandardDeviationPoseEstimationDoubleMarker"];
    current = fn.begin(), it_end = fn.end();
    for (; current != it_end; ++current)
    {
        vector<double> tmp;
        FileNode item = *current;
        item >> tmp;
        StdDevDoubleMarker.push_back(tmp);
    }
    
    fs.release();
}

void LocalizationCv::getObjectAndImagePoints(InputArrayOfArrays detectedCorners,
InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints) {

    size_t nDetectedMarkers = detectedIds.total();

    vector< Point3f > objPnts;
    objPnts.reserve(nDetectedMarkers);

    vector< Point2f > imgPnts;
    imgPnts.reserve(nDetectedMarkers);

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
}

double LocalizationCv::getMinDetectionPerimeter(int id)
{
    return 4 * cameraf / maxDetectDistance * marker.getMarkersize(id);
}

void LocalizationCv::removeOutofRange(std::vector<int> & ids, std::vector<std::vector<cv::Point2f> > & corners, std::vector<double> & perimeters)
{
    std::vector<int> idscopys = ids; 
    ids.clear();
    std::vector<std::vector<cv::Point2f> > cornerscopys = corners;
    corners.clear();
    std::vector<double> perimeterscopys = perimeters;
    perimeters.clear();

    for (int i = 0; i < idscopys.size(); ++i)
    {   
        double per = getMinDetectionPerimeter(idscopys[i]);

        if(perimeterscopys[i] >= per)
        {
            ids.push_back(idscopys[i]);
            corners.push_back(cornerscopys[i]);
            perimeters.push_back(perimeterscopys[i]);
        }
    }
}

tf2::Transform LocalizationCv::worldToUTM(const tf2::Transform & tf)
{   
    tf2::Vector3 vv = tf.getOrigin();
    tf2::Quaternion qqtf2 = tf.getRotation();

    tf::Quaternion qq(qqtf2.x(), qqtf2.y(), qqtf2.z(), qqtf2.w());
    tf::Matrix3x3 Mr;
    Mr.setRotation(qq);
    double yaw, pitch, roll;
    Mr.getEulerYPR(yaw, pitch, roll);
    double x = vv.getX();
    double y = vv.getY();

    yaw = -yaw + M_PI/2;

    while (yaw> 2*M_PI) yaw -= 2.*M_PI;
    while (yaw<-0) yaw += 2.*M_PI;

    double xutm1 = 606167.51; 
    double xutm2 = 606198.75;
    double yutm1 = 5797146.69;
    double yutm2 = 5797156.54;

    double xworld1 = 0;
    double xworld2 = 0;
    double yworld1 = 0;
    double yworld2 = 32.71;

    double dx = xutm1 - xworld1;
    double dy = yutm1 - yworld1;
    double arutm = (yutm1 - yutm2) / (xutm1 - xutm2);
    double arworld = (yworld1 - yworld2) / (xworld1 - xworld2);
    double mu = atan(arutm);
    double ml = atan(arworld);

    double phi;
    if(ml < 0 && mu > 0)
        phi = -(M_PI + ml - mu);
    else if(ml < 0 && mu < 0)
        phi = -(-ml + mu);
    else
        phi = -(ml - mu);

    MatrixXd DM(2,2);
    DM << cos(phi), -sin(phi), sin(phi), cos(phi);

    MatrixXd PW(2,1);
    PW << x,y;

    MatrixXd PW_A = DM*PW;

    double xA = PW_A(0,0);
    double yA = PW_A(1,0);
    double yawA = yaw - phi;

    while (yawA> 2*M_PI) yawA -= 2.*M_PI;
    while (yawA<-0) yawA += 2.*M_PI;

    double xA2 = xA + dx;
    double yA2 = yA + dy;

    tf2::Vector3 vout = tf2::Vector3(xA2, yA2, yawA);
    tf2::Quaternion qout;

    tf2::Transform Tout(qout, vout);

    return Tout;
}


Eigen::Affine2d LocalizationCv::Find2DAffineTransform(Eigen::Matrix2Xd in, Eigen::Matrix2Xd out) {
  // Default output
  Eigen::Affine2d A;
  A.linear() = Eigen::Matrix2d::Identity(2, 2);
  A.translation() = Eigen::Vector2d::Zero();

  if (in.cols() != out.cols())
    throw "Find2DAffineTransform(): input data mis-match";

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

// The input 3D points are stored as columns.
Eigen::Affine3d LocalizationCv::Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {

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

void LocalizationCv::mysolvepnp(const cv::Mat &objPoints, const cv::Mat &imgPoints, const cv::Mat &cameraMatrix, Eigen::AngleAxisd &rvec, Eigen::Vector3d &tvec)
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
    std::cout<<"eigen矩阵是："<<input_mat_obj<<std::endl;
    double x1,y1,z1,u1,v1,x2,y2,z2,u2,v2;
    double k1,k2;
    double g1,g2;
    double fx,cx,fy,cy;
    double H1,H2;

    fx = cameraMatrix.at<double>(0,0);
    cx = cameraMatrix.at<double>(0,2);
    fy = cameraMatrix.at<double>(1,1);
    cy = cameraMatrix.at<double>(1,2);

       for (int i=0; i<img.size();i++)
    {
        input_mat_img(2,i) = (1.3114-obj[i].z)*fy/(img[i].y-cy);
        input_mat_img(1,i) = 1.3114-obj[i].z;
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
    t(1)=1.3114;
    t(2)=B.translation()(1);
    std::cout<<"R: "<<R<<std::endl;
    std::cout<<"t: "<<t<<std::endl;
    
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(R);
    //std::cout<<"rvec: "<<rotation_vector.axis()<<std::endl;
    //std::cout<<"theta: "<<rotation_vector.angle()<<std::endl;
    rvec = rotation_vector;
    tvec = t;

    x1 = objPoints.at<float>(0,0);
    y1 = objPoints.at<float>(0,1);
    z1 = objPoints.at<float>(0,2);
    u1 = imgPoints.at<float>(0,0);
    v1 = imgPoints.at<float>(0,1);

    x2 = objPoints.at<float>(5,0);
    y2 = objPoints.at<float>(5,1);
    z2 = objPoints.at<float>(5,2);
    u2 = imgPoints.at<float>(5,0);
    v2 = imgPoints.at<float>(5,1);
    //std::cout<<imgPoints.ptr< float >(0)[1]<<std::endl;
    //t1 = obj[0].x;
    //t2 = img[0].y;
}
bool LocalizationCv::getPoseVariance(const std::vector<int> &ids, const std::vector<double> &perimeters, std::vector<double> &Var)
{   
    Var.clear();
    Var.resize(3);
    if(ids.size() == 1)
    {   
        if (StdDevSingleMarker.empty())
            return false;

        double lside = perimeters[0] / 4.0;
        int index = -1;
        for (int i=0; i < StdDevSingleMarker[0].size(); ++i)
        {   
            if (lside >= StdDevSingleMarker[0][i])
            {
                index = i;
                break;
            }
        }
        if (index == -1)
        {
            Var[0] = 100;
            Var[1] = 100;
            Var[2] = 100;
        }
        else
        {
            Var[0] = StdDevSingleMarker[1][index];
            Var[1] = StdDevSingleMarker[2][index];
            Var[2] = StdDevSingleMarker[3][index];
        }
    }
    else
    {
        if (StdDevDoubleMarker.empty())
            return false;
        double lside = perimeters.back() / 4.0;
        int index = -1;
        for (int i=0; i < StdDevDoubleMarker[0].size(); ++i)
        {   
            if (lside >= StdDevDoubleMarker[0][i])
            {
                index = i;
                break;
            }
        }
        if (index == -1)
        {
            Var[0] = 100;
            Var[1] = 100;
            Var[2] = 100;
        }
        else
        {
            Var[0] = StdDevDoubleMarker[1][index];
            Var[1] = StdDevDoubleMarker[2][index];
            Var[2] = StdDevDoubleMarker[3][index];
        }
    }
    return true;
}

void LocalizationCv::imageCb(const sensor_msgs::ImageConstPtr& msg)
{   
    //ROS_INFO_STREAM("[imageCb thread=" << boost::this_thread::get_id() << "]");
    cv_bridge::CvImagePtr cv_ptr;

    //@ show warning if image in the callback queue is drop
    int dseq = msg->header.seq - preSequenceId;
    if (dseq != 1)
        ROS_WARN("Image in the callback queue was dropped");
    preSequenceId = msg->header.seq;

    try
    { 
        //@ make a copy of msg,returning a mutable CvImage(boost::shared_ptr)
        //@ "cv_ptr->image" equivalent to "cv::Mat"
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::YUV422);
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //@ convert the YUV color space to BGR
    if (msg->encoding == sensor_msgs::image_encodings::YUV422)
    {
        cv::Mat imageBGR8;
        cv::cvtColor(cv_ptr->image, imageBGR8, cv::COLOR_YUV2BGR_UYVY);
        cv_ptr->image = imageBGR8;
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    }
    
    //@ detected marker ID list 
    std::vector<int> ids;
    //@ 2D points specified by its coordinates x and y,Point2f - float
    std::vector<std::vector<cv::Point2f> > corners;

    double mpsize = cameraf * minmarkersize / maxDetectDistance;
    double impre = std::max(cv_ptr->image.cols, cv_ptr->image.rows);
    parameters->minMarkerPerimeterRate = 4 * mpsize / impre;
    parameters->maxMarkerPerimeterRate = 4;
    //parameters->minMarkerDistanceRate = 0.05;

    //@ detect markers in entire image or in ROI of image
    if (!ROIvec.empty())
    {
        cv::Mat imageROI = cv_ptr->image.clone();

        cv::Range ROIu = cv::Range(ROIvec[0],ROIvec[1]); // x
        cv::Range ROIv = cv::Range(ROIvec[2],ROIvec[3]);  // y

        imageROI = imageROI(ROIv, ROIu);

        cv::aruco::detectMarkers(imageROI, dictionary, corners, ids, parameters);

        for (int i = 0; i < corners.size(); ++i)
        {   
            for (int j = 0; j < corners[i].size(); ++j)
            {
                corners[i][j].x = corners[i][j].x + ROIu.start;
                corners[i][j].y = corners[i][j].y + ROIv.start;
            }
            
        }

    }
    else
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, parameters);

    //@ draw all detected markers with green border
    if (ids.size() > 0)
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    //@ Remove ids and corners that do not belong to Map
    std::vector<int> idscopys = ids; 
    ids.clear();
    std::vector<std::vector<cv::Point2f> > cornerscopys = corners;
    corners.clear();
    for (int i = 0; i < idscopys.size(); ++i)
    {
        std::unordered_map <int, std::vector<double> >::iterator iter1;
        iter1 = marker.marker_pose.find(idscopys[i]);
        if(iter1 != marker.marker_pose.end())
        {
            ids.push_back(idscopys[i]);
            corners.push_back(cornerscopys[i]);
            //set_pose.push_back(marker_pose);
        }
    }

    //@ Sort detected markers according to the size of the marker in image (big->small)
    std::vector<double> idsperimeter;
    for (int i = 0; i < ids.size(); ++i)
    {   
        //@ marker perimeter in pixel
        double p = cv::arcLength(corners[i],true);
        idsperimeter.push_back(p);
    }
    if(ids.size() > 1)
    {
        for (int i = 0; i < ids.size()-1; ++i)
        {   
            for (int j = 0; j < ids.size()-i-1; ++j)
            {
                if(idsperimeter[j] < idsperimeter[j+1])
                {   
                    std::swap(idsperimeter[j],idsperimeter[j+1]);
                    std::swap(ids[j],ids[j+1]);
                    std::swap(corners[j],corners[j+1]);
                }
            }
        }
        
    }


    //@ Remove markers beyond the range according to the perimeter
    //@ The threshold is given by parameter "maxDetectDistance"
    removeOutofRange(ids,corners,idsperimeter);

    //@ Remove relatively small markers
    //@ The threshold is given by parameter "RelativeMarkerSizeToSolvePnP"
    std::vector<int> idscopy = ids; 
    ids.clear();
    std::vector<std::vector<cv::Point2f> > cornerscopy = corners;
    corners.clear();
    std::vector<double> perimeterscopys = idsperimeter;
    idsperimeter.clear();
    for (int i = 0; i < idscopy.size(); ++i)
    {
        if(i == 0)
        {
            ids.push_back(idscopy[i]);
            corners.push_back(cornerscopy[i]);
            idsperimeter.push_back(perimeterscopys[i]);
        }
        else if( idsperimeter[i] / idsperimeter[0] > RelativeMarkerSizeToSolvePnP)
        {
            ids.push_back(idscopy[i]);
            corners.push_back(cornerscopy[i]);
            idsperimeter.push_back(perimeterscopys[i]);
        }
    }

    //@ Whether to use a single marker for pose estimation or not
    if (ids.size() == 1 && !LocalizationWithSingleMarker)
        ids.clear();

    //@ draw the markers to pnp with red border
    if (ids.size() > 0)
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids, cv::Scalar(0,0,255));

    //@ Output modified video stream for visualization
    image_pub_.publish(cv_ptr->toImageMsg());

    //@ *************sovle PnP_Problem: calculate Tcw*************
    //cv::Vec3d rvec, tvec;
    Eigen::AngleAxisd rvec;
    Eigen::Vector3d tvec;
    //cv::OutputArray inarr;
    Mat objPoints, imgPoints;
    double theta,t1,t2;
    if(ids.size()>0)
    {   
        int flag = 0;
        if (ids.size() == 1)
            flag = 0;
        
        getObjectAndImagePoints(corners, ids, objPoints, imgPoints);
        if (objPoints.size() == imgPoints.size() && !objPoints.empty())
        {
            //solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs,rvec, tvec,false,6);
            //solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs,rvec, tvec,false,flag);
            //solvePnPRansac(objPoints, imgPoints, cameraMatrix, distCoeffs,rvec, tvec,false,100,8.0,0.99,noArray(),6);
            mysolvepnp(objPoints,imgPoints,cameraMatrix,rvec, tvec);
            //std::cout<<t1<<" "<<t2<<std::endl;
        }
            //sol
    }
    std::cout<<objPoints<<"++ "<<imgPoints<<std::endl;
    //@ ***********remove outliers of pose *************
    PosePnPResult currpositon;
    currpositon.time = ros::Time::now();
    //currpositon.vt = tvec;
    //currpositon.rt = rvec;
    bool isnotoutliers = true;

    if(ids.size()>0)
    {   
        isnotoutliers = IsOutlier_Hight(currpositon, RemoveOutlierHightThreshold, RemoveOutlierHight);
        if ( isnotoutliers == true )
        {
            if (prePose.time.toSec() == 0)
                prePose = currpositon;
            else
            {
                isnotoutliers = IsOutlier_Speed(prePose, currpositon, RemoveOutlierSpeedThreshold, RemoveOutlierSpeed);
                if (isnotoutliers)
                    prePose = currpositon;
            }
        }   
    }

    //@ *************publish topics *********************
    geometry_msgs::PoseStamped VehiclePoseSt;
    geometry_msgs::PoseStamped VehiclePoseSt_UTM;
    geometry_msgs::PoseWithCovarianceStamped VehiclePoseStCov;
    geometry_msgs::PoseWithCovarianceStamped VehiclePoseStCov_UTM;

 //   VehiclePoseSt.header.seq = msg->header.seq;
 //   VehiclePoseSt.header.stamp = msg->header.stamp;
 //   VehiclePoseSt.header.frame_id = "world";
 //   VehiclePoseSt.pose.position.x = t1;
 //   VehiclePoseSt.pose.position.y = t2;
 //   VehiclePoseSt.pose.position.z = 1.3114;
    //tf::Quaternion q;
    //q=tf::createQuaternionMsgFromYaw(theta+M_PI/2);
  //  VehiclePoseSt.pose.orientation.x = q_vehicle_pose_in_world.x();
  //  VehiclePoseSt.pose.orientation.y = q_vehicle_pose_in_world.y();
  //  VehiclePoseSt.pose.orientation.z = q_vehicle_pose_in_world.z();
  //  VehiclePoseSt.pose.orientation.w = q_vehicle_pose_in_world.w();
  //  VehiclePoseSt.pose.orientation = tf::createQuaternionMsgFromYaw(theta+M_PI/2);
   // pose_pub_.publish(VehiclePoseSt);
    isnotoutliers = 1;
    static tf2_ros::TransformBroadcaster br; //? static
    //@ if at least one marker detected and the pose is not an outliers
    if (ids.size() > 0 && !objPoints.empty() && isnotoutliers)
    {   
        //@ normalazition of rotation vector
       // std::vector<double> rv = {rvec(0),rvec(1),rvec(2)};
        //double norm = sqrt(pow(rv[0],2)+pow(rv[1],2)+pow(rv[2],2));
       // for (int i = 0; i < rv.size(); ++i)
       // {
       //     rv[i] = rv[i]/norm;
       // }

        tf2::Vector3 tf2rv =tf2::Vector3(rvec.axis()(0),rvec.axis()(1),rvec.axis()(2));

        //@ construct quaternion with rotation vector
        tf2::Quaternion Q(tf2rv, rvec.angle());
    
        tf2::Vector3 v_marker_pose_in_cam = { tvec(0), tvec(1), tvec(2)};
        tf2::Quaternion q_marker_pose_in_cam = Q;

        //@ Tcw(world coordinate pose in camera coordinate system)
        tf2::Transform trans_cam_world(q_marker_pose_in_cam, v_marker_pose_in_cam);
        //@ Twc(camera pose in world coordinate system)
        tf2::Transform trans_world_cam = trans_cam_world.inverse();
        //@ convert tf2 data to geometry_msgs data
        geometry_msgs::Transform msgs_trans_world_cam = tf2::toMsg(trans_world_cam);
        std::cout<<msgs_trans_world_cam<<std::endl; 
        //@ send transformation of camera to tf2 (for visualization)
        geometry_msgs::TransformStamped transformStampedCam;
        transformStampedCam.header.stamp = ros::Time::now();
        transformStampedCam.header.frame_id = "world";
        transformStampedCam.child_frame_id = "camera";
        transformStampedCam.transform = msgs_trans_world_cam;
        br.sendTransform(transformStampedCam);

        //@ transform_vc(vehicle camera)(Tvc)
        tf2::Vector3 Vvc =tf2::Vector3(Transform_vc[0], Transform_vc[1], Transform_vc[2]);
        tf2::Quaternion Qvc;
        Qvc.setRPY(Transform_vc[3], Transform_vc[4], Transform_vc[5]);
        tf2::Transform trans_vehicle_cam(Qvc, Vvc);

        //@ transform_wv(world vehicle)(Twv)
        tf2::Transform trans_world_vehicle = trans_world_cam * trans_vehicle_cam.inverse();
        tf2::Vector3 v_vehicle_pose_in_world = trans_world_vehicle.getOrigin();
        tf2::Quaternion q_vehicle_pose_in_world = trans_world_vehicle.getRotation();
        geometry_msgs::Transform msgs_trans_world_vehicle = tf2::toMsg(trans_world_vehicle);

        std::vector<double> variance;
        bool hascov = getPoseVariance(ids,idsperimeter,variance);

        boost::array<double,36> cov = {};
        if (hascov)
        {
            cov[0] = pow(variance[0],2);
            cov[7] = pow(variance[1],2);
            cov[35] = pow(variance[2] / 180.0 * M_PI, 2);
        }

        VehiclePoseStCov.pose.covariance = cov;

        VehiclePoseSt.header.seq = msg->header.seq;
        VehiclePoseSt.header.stamp = msg->header.stamp;
        VehiclePoseSt.header.frame_id = "world";
        VehiclePoseSt.pose.position.x = v_vehicle_pose_in_world.getX();
        VehiclePoseSt.pose.position.y = v_vehicle_pose_in_world.getY();
        VehiclePoseSt.pose.position.z = v_vehicle_pose_in_world.getZ();
       // VehiclePoseSt.pose.position.x = t1;
        //VehiclePoseSt.pose.position.y = t2;
        //VehiclePoseSt.pose.position.z = 1.3114;
        VehiclePoseSt.pose.orientation.x = q_vehicle_pose_in_world.x();
        VehiclePoseSt.pose.orientation.y = q_vehicle_pose_in_world.y();
        VehiclePoseSt.pose.orientation.z = q_vehicle_pose_in_world.z();
        VehiclePoseSt.pose.orientation.w = q_vehicle_pose_in_world.w();
        pose_pub_.publish(VehiclePoseSt);

        VehiclePoseStCov.header = VehiclePoseSt.header;
        VehiclePoseStCov.pose.pose = VehiclePoseSt.pose;
        pose_pub_cov_.publish(VehiclePoseStCov);

        //@ send transformation of vehicle to tf (for visualization)
        geometry_msgs::TransformStamped transformStampedVeh;
        transformStampedVeh.header.stamp = ros::Time::now();
        transformStampedVeh.header.frame_id = "world";
        transformStampedVeh.child_frame_id = "vehicle";
        transformStampedVeh.transform = msgs_trans_world_vehicle;
        br.sendTransform(transformStampedVeh);

        //@UTM_transform_wv(world vehicle)(Twv)
        tf2::Transform trans_UTM_vehicle = worldToUTM(trans_world_vehicle);
        tf2::Vector3 v_vehicle_UTM = trans_UTM_vehicle.getOrigin();
        tf2::Quaternion q_vehicle_UTM = trans_UTM_vehicle.getRotation();

        VehiclePoseSt_UTM.header.seq = msg->header.seq;
        VehiclePoseSt_UTM.header.stamp = msg->header.stamp;
        VehiclePoseSt_UTM.header.frame_id = "UTM";
        VehiclePoseSt_UTM.pose.position.x = v_vehicle_UTM.getX();
        VehiclePoseSt_UTM.pose.position.y = v_vehicle_UTM.getY();
        //@ position.z is yaw angle in deg
        VehiclePoseSt_UTM.pose.position.z = v_vehicle_UTM.getZ() * 180.0 / M_PI;
        // VehiclePoseSt_UTM.pose.orientation.x = q_vehicle_pose_in_world.x();
        // VehiclePoseSt_UTM.pose.orientation.y = q_vehicle_pose_in_world.y();
        // VehiclePoseSt_UTM.pose.orientation.z = q_vehicle_pose_in_world.z();
        // VehiclePoseSt_UTM.pose.orientation.w = q_vehicle_pose_in_world.w();

        pose_pub_utm_.publish(VehiclePoseSt_UTM);

        cov[35] = cov[35] * pow(180.0 / M_PI,2);
        VehiclePoseStCov_UTM.pose.covariance = cov;
        VehiclePoseStCov_UTM.header = VehiclePoseSt_UTM.header;
        VehiclePoseStCov_UTM.pose.pose = VehiclePoseSt_UTM.pose;
        pose_pub_cov_utm_.publish(VehiclePoseStCov_UTM);

        //@ send the pose of detected marker to ros tf
        for (int i = 0; i < ids.size(); ++i)
        {
            //@ find the detected id in map
            std::unordered_map <int, std::vector<double> >::iterator iter1;
            iter1 = marker.marker_pose.find(ids[i]);

            //@ If a marker is found, then transform
            if(iter1 != marker.marker_pose.end())
            {
                std::vector<double> marker_pose_temp = iter1->second;

                tf2::Vector3 v_marker_pose_in_world = {marker_pose_temp[0],
                  marker_pose_temp[1],marker_pose_temp[2]};

                tf2::Quaternion q_marker_pose_in_world = { marker_pose_temp[3],
                  marker_pose_temp[4],marker_pose_temp[5],marker_pose_temp[6]};

                //@ send transformation of detected marker to tf2 (for visualization)
                geometry_msgs::TransformStamped transformStampedMr;
                transformStampedMr.header.stamp = ros::Time::now();
                transformStampedMr.header.frame_id = "world";
                transformStampedMr.child_frame_id = "Marker_id "+ std::to_string(ids[i]);
                transformStampedMr.transform.translation.x = v_marker_pose_in_world[0];
                transformStampedMr.transform.translation.y = v_marker_pose_in_world[1];
                transformStampedMr.transform.translation.z = v_marker_pose_in_world[2];
                transformStampedMr.transform.rotation.x = q_marker_pose_in_world.x();
                transformStampedMr.transform.rotation.y = q_marker_pose_in_world.y();
                transformStampedMr.transform.rotation.z = q_marker_pose_in_world.z();
                transformStampedMr.transform.rotation.w = q_marker_pose_in_world.w();
                br.sendTransform(transformStampedMr);
            }
        }
    }
}

// void LocalizationCv::imageCbthread(const sensor_msgs::ImageConstPtr& msg)
// {
//     boost::thread imageCbfunc( boost::bind(&LocalizationCv::imageCb,this,msg));
//     imageCbfunc.join();
//     //imageCbfunc.try_join_for(boost::chrono::milliseconds(20));
//     ROS_INFO_STREAM("[imageCbthread thread=" << boost::this_thread::get_id() << "]");
//     //count++;
// }

#endif
