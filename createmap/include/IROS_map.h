//
// Created by zzh on 2019/12/19.
//

#ifndef CREATEMAP_IROS_MAP_H
#define CREATEMAP_IROS_MAP_H
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
namespace IROS {
    struct IMUDATA {

        /// Timestamp of the reading
        double timestamp;

        /// Gyroscope reading, angular velocity (rad/s)
        Eigen::Matrix<double, 3, 1> wm;

        /// Accelerometer reading, linear acceleration (m/s^2)
        Eigen::Matrix<double, 3, 1> am;

    };
    struct ACC {
        double timestamp;
        Eigen::Vector3d am;
    };
    struct GYR {
        double timestamp;
        Eigen::Vector3d wm;
    };
    struct POSE {
        Eigen::Matrix3d rotation;
        Eigen::Vector3d transpose;
    };
    struct PIC {
        double timestamp;
        cv::Mat image;
    };
    GYR inter_gyr(const GYR& gyr_1, const GYR gyr_2, double timestamp){
        double lambda = (timestamp - gyr_1.timestamp) / (gyr_2.timestamp - gyr_1.timestamp);
        GYR gyr;
        gyr.timestamp = timestamp;
        gyr.wm = (1 - lambda) * gyr_1.wm + lambda * gyr_2.wm;
        return gyr;
    }
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
    void imuconfig_from_yaml(ros::NodeHandle &n, vi_map::ImuSigmas &imu_sigma) {
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

    void camera_config_from_yaml(ros::NodeHandle &n, Eigen::Matrix<double, 4, 1> &cam_proj,
                                 Eigen::Matrix<double, 4, 1> &cam_dist, Eigen::Matrix4d &T_I_C,
                                 std::vector<int> &resolution) {
        std::string config_file;
        std::string config_default = "/home/zzh/rosbag/ss/OpenLORIS.yaml";
        n.param<std::string>("config_file", config_file, config_default);
        cv::FileStorage fsSetting(config_file, cv::FileStorage::READ);
        //camera neican
        cv::FileNode proj_para = fsSetting["projection_parameters"];
        cam_dist << proj_para["k2"], proj_para["k3"], proj_para["k4"], proj_para["k5"];
        cam_proj << proj_para["mu"], proj_para["mv"], proj_para["u0"], proj_para["v0"];
        //camera waican
        cv::Mat rotation, transpose;
        Eigen::Matrix3d rotation_eigen;
        Eigen::Vector3d transpose_eigen;
        fsSetting["C02IMUextrinsicRotation"] >> rotation;
        fsSetting["C02IMUextrinsicTranslation"] >> transpose;
        cv::cv2eigen(rotation, rotation_eigen);
        cv::cv2eigen(transpose, transpose_eigen);
        T_I_C.block(0, 0, 3, 3) = rotation_eigen;
        T_I_C.block(0, 3, 3, 1) = transpose_eigen;
        //fenbianlv
        int width = fsSetting["image_width"];
        int height = fsSetting["image_height"];
        resolution.push_back(width);
        resolution.push_back(height);
        fsSetting.release();
    }
    void get_topic_and_path(ros::NodeHandle& n, std::string& cam_topic,
                            std::string& acc_topic,std::string& gyr_topic, std::string& gt_topic, std::string& map_folder){
        std::string cam_topic_default = "/cam0/image_raw";
        std::string acc_topic_default = "/imu0";
        std::string gyr_topic_default = "";
        std::string gt_topic_default = "/tf0";
        std::string map_folder_default = "/home/zzh/_map";
        n.param<std::string>("camera_topic", cam_topic, cam_topic_default);
        n.param<std::string>("acc_topic", acc_topic, acc_topic_default);
        n.param<std::string>("gyr_topic", gyr_topic, gyr_topic_default);
        n.param<std::string>("gt_topic", gt_topic, gt_topic_default);
        n.param<std::string>("map_folder", map_folder, map_folder_default);
    }
}
#endif //CREATEMAP_IROS_MAP_H
