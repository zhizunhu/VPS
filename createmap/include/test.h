//
// Created by zzh on 2019/12/14.
//

#ifndef CREATEMAP_TEST_H
#define CREATEMAP_TEST_H
//
// Created by zzh on 2019/12/11.
//

#ifndef CREATEMAP_MAPBUILDER_H
#define CREATEMAP_MAPBUILDER_H

#include <ros/ros.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <vio-common/vio-types.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include <online-map-builders/stream-map-builder.h>
#include <feature-tracking/vo-feature-tracking-pipeline.h>
#include <vi-map/vi-map-serialization.h>
#include <opencv2/core/eigen.hpp>
namespace test {
    typedef Eigen::Matrix<double, 3, 1> Vector3;
    struct IMUDATA {

        /// Timestamp of the reading
        double timestamp;

        /// Gyroscope reading, angular velocity (rad/s)
        Eigen::Matrix<double, 3, 1> wm;

        /// Accelerometer reading, linear acceleration (m/s^2)
        Eigen::Matrix<double, 3, 1> am;

    };
    struct pose{
        Eigen::Matrix3d rotation;
        Eigen::Vector3d transpose;
    };
    struct pic_frame{
        double timestamp;
        cv::Mat image;
    };
    IMUDATA interpolate_data(const IMUDATA imu_1, const IMUDATA imu_2, double timestamp) {
        // time-distance lambda
        double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
        //cout << "lambda - " << lambda << endl;
        // interpolate between the two times
        IMUDATA data;
        data.timestamp = timestamp;
        data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
        data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
        return data;
    }
    bool isLessThenEpsilons4thRoot(double x){
        static const double epsilon4thRoot = pow(std::numeric_limits<double>::epsilon(), 1.0/4.0);
        return x < epsilon4thRoot;
    }
    double arcSinXOverX(double x) {
        if(isLessThenEpsilons4thRoot(fabs(x))){
            return double(1.0) + x * x * double(1.0/6.0);
        }
        return asin(x) / x;
    }
    Eigen::Vector3d log(const Eigen::Quaterniond &q){
        const Eigen::Matrix<double, 3, 1> a = Vector3(q.x(),q.y(),q.z());
        const double na = a.norm();
        const double eta = q.w();
        double scale;
        if(fabs(eta) < na){ // use eta because it is more precise than na to calculate the scale. No singularities here.
            // check sign of eta so that we can be sure that log(-q) = log(q)
            if (eta >= 0) {
                scale = acos(eta) / na;
            } else {
                scale = -acos(-eta) / na;
            }
        } else {
            if(eta > 0) {
                // For asin(na)/ na the singularity na == 0 can be removed. We can ask (e.g. Wolfram alpha) for its series expansion at na = 0. And that is done in the following function.
                scale = arcSinXOverX(na);
            } else {
                // the negative is here so that log(-q) == log(q)
                scale = arcSinXOverX(na);
            }
        }
        return a * (double(2.0) * scale);
    }
    Eigen::Quaterniond exp(const Vector3 & dx){
        // Method of implementing this function that is accurate to numerical precision from
        // Grassia, F. S. (1998). Practical parameterization of rotations using the exponential map. journal of graphics, gpu, and game tools, 3(3):29–48.
        double theta = dx.norm();
        // na is 1/theta sin(theta/2)
        double na;
        if(isLessThenEpsilons4thRoot(theta)){
            static const double one_over_48 = 1.0/48.0;
            na = 0.5 + (theta * theta) * one_over_48;
        } else {
            na = sin(theta*0.5) / theta;
        }
        double ct = cos(theta*0.5);
        return Eigen::Quaterniond(ct,dx[0]*na,dx[1]*na,dx[2]*na);
    }
    pose interplate_gt(const pose pose1, double timestamp1, const pose pose2, double timestamp2, double cur_timestamp){
        pose cur_pose;
        double lambda = (cur_timestamp - timestamp1) / (timestamp2 - timestamp1);
        cur_pose.transpose = (1 - lambda) * pose1.transpose + lambda * pose2.transpose;

        Eigen::Quaterniond q_befor(pose1.rotation);
        Eigen::Quaterniond q_after(pose2.rotation);
        Eigen::Quaterniond q_cur;
        q_cur.setIdentity();
        Eigen::Quaterniond q_mid = q_befor.inverse() * q_after;
        q_cur = q_befor * exp(lambda * log(q_mid));
        cur_pose.rotation = q_cur.toRotationMatrix();

        return cur_pose;
    }
    void imuconfig_from_yaml(ros::NodeHandle& n, vi_map::ImuSigmas& imu_sigma){
        std::string config_file;
        std::string config_default = "/home/zzh/rosbag/ss/OpenLORIS.yaml";
        n.param<std::string>("config_file", config_file, config_default);
        cv::FileStorage fsSetting(config_file, cv::FileStorage::READ);
        //imu canshu
        imu_sigma.acc_noise_density = fsSetting["acc_n"];
        imu_sigma.acc_bias_random_walk_noise_density = fsSetting["acc_w"];
        imu_sigma.gyro_noise_density = fsSetting["gyr_n"];
        imu_sigma.gyro_bias_random_walk_noise_density = fsSetting["gyr_w"];
        fsSetting.release();
    }
    void camera_config_from_yaml(ros::NodeHandle& n, Eigen::Matrix<double, 4, 1>& cam_proj,
                                 Eigen::Matrix<double, 4, 1>& cam_dist, Eigen::Matrix4d& T_I_C, std::vector<int>& resolution){
        std::string config_file;
        std::string config_default = "home/zzh/maplab_src/src/createmap/config/cameraandimu.yaml";
        n.param<std::string>("config_file", config_file, config_default);
        cv::FileStorage fsSetting(config_file, cv::FileStorage::READ);
        //camera neican
        if (!fsSetting.isOpened()){
            std::cout << "faild"  << std::endl;
        }
        cv::FileNode proj_para = fsSetting["projection_parameters"];
        cam_dist << proj_para["k2"], proj_para["k3"], proj_para["k4"], proj_para["k5"];
        cam_proj << proj_para["mu"], proj_para["mv"], proj_para["u0"], proj_para["v0"];
        //camera waican
        cv::Mat rotation, transpose;
        Eigen::Matrix3d rotation_eigen;
        Eigen::Vector3d transpose_eigen;
        fsSetting["C02IMUextrinsicRotation"] >>  rotation;
        fsSetting["C02IMUextrinsicTranslation"] >> transpose;
        cv::cv2eigen(rotation, rotation_eigen);
        cv::cv2eigen(transpose, transpose_eigen);
        T_I_C.block(0,0,3,3)= rotation_eigen;
        T_I_C.block(0, 3, 3, 1)= transpose_eigen;
        //fenbianlv
        int width = fsSetting["image_width"];
        int height = fsSetting["image_height"];
        resolution.push_back(width);
        resolution.push_back(height);
        fsSetting.release();
    }
    void get_topic_and_path(ros::NodeHandle& n, std::string& cam_topic,
                            std::string& imu_topic, std::string& gt_path, std::string& map_folder){
        std::string cam_topic_default = "/cam0/image_raw";
        std::string imu_topic_default = "/imu0";
        std::string gt_path_default = "/home/zzh/sharetohost/cafe2_gt.txt";
        std::string map_folder_default = "/home/zzh/_map";
        n.param<std::string>("camera_topic", cam_topic, cam_topic_default);
        n.param<std::string>("imu_topic", imu_topic, imu_topic_default);
        n.param<std::string>("gt_path", gt_path, gt_path_default);
        n.param<std::string>("map_folder", map_folder, map_folder_default);
    }
}
#endif //CREATEMAP_MAPBUILDER_H

#endif //CREATEMAP_TEST_H
