//
// Created by zzh on 2019/12/13.
//
//
// Created by zzh on 2019/12/11.
//
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
#include <test.h>

std::vector<test::IMUDATA> imu_data;
std::vector<test::groudtruth> gt_data;
vi_map::VIMap* map;
online_map_builders::StreamMapBuilder* builder;
std::shared_ptr<aslam::NCamera> camera_rig;
double last_timestamp = -1;
double cur_timestamp = -1;
double next_timestamp = -1;
std::shared_ptr<aslam::NCamera> get_camera_rig(ros::NodeHandle n);
int main(int argc, char ** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle n("~");
    vi_map::ImuSigmas imu_sigma;
    test::imuconfig_from_yaml(n, imu_sigma);

    constexpr char KImuHardwareID[] = "imu0";
    vi_map::SensorId imu_sensor_Id;
    common::generateId(&imu_sensor_Id);
    vi_map::Imu::UniquePtr imu_rig = aligned_unique<vi_map::Imu>(imu_sensor_Id, static_cast<std::string>(KImuHardwareID));
    imu_rig->setImuSigmas(imu_sigma);

    camera_rig = get_camera_rig(n);
    std::string cam_topic;
    std::string imu_topic;
    std::string gt_topic;
    std::string map_folder;
    test::get_topic_and_path(n, cam_topic, imu_topic, gt_topic, map_folder);
    map = new vi_map::VIMap(map_folder);
    builder = new online_map_builders::StreamMapBuilder(camera_rig, std::move(imu_rig), map);

    std::string path_to_bag_default = "/home/zzh/sharetohost/rosbag/";
    std::string path_to_bag;
    n.param("path_to_bag", path_to_bag, path_to_bag_default);
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    rosbag::View view_imu(bag, rosbag::TopicQuery(imu_topic));
    rosbag::View view_pic(bag, rosbag::TopicQuery(cam_topic));
    rosbag::View view_gt(bag, rosbag::TopicQuery(gt_topic));
    std::vector<test::IMUDATA> imu_data;
    if(view_imu.size() == 0){
        ROS_ERROR("open bag file failed");
        return -1;
    }
    int i = 0;
    //读取IMU数据
    for (const rosbag::MessageInstance& m : view_imu){
        if (!ros::ok()){
            break;
        }
        sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        double timem = (*imu).header.stamp.toSec();
        Eigen::Matrix<double, 3, 1> wm, am;
        wm << (*imu).angular_velocity.x, (*imu).angular_velocity.y, (*imu).angular_velocity.z;
        am << (*imu).linear_acceleration.x, (*imu).linear_acceleration.y, (*imu).linear_acceleration.z;
        test::IMUDATA data;
        data.timestamp = timem;
        data.am = am;
        data.wm = wm;
        imu_data.push_back(data);
        i ++ ;
    }

    std::vector<test::groudtruth> gt_all;
    for (const rosbag::MessageInstance & m : view_gt){
        if (!ros::ok()){
            break;
        }
        geometry_msgs::PoseStamped::ConstPtr gt = m.instantiate<geometry_msgs::PoseStamped>();
        double timeg = (*gt).header.stamp.toSec();
        Eigen::Quaterniond q((*gt).pose.orientation.w, (*gt).pose.orientation.x, (*gt).pose.orientation.y, (*gt).pose.orientation.z);
        Eigen::Vector3d t;
        t << (*gt).pose.position.x, (*gt).pose.position.y, (*gt).pose.position.z;
        test::groudtruth gt_1;
        gt_1.timestamp = timeg;
        gt_1.rotation = q.toRotationMatrix();
        gt_1.transpose = t;
        gt_all.push_back(gt_1);
    }
    double last_timestamp = -1;
    //调节频率
    bool decframe = true;
    int gt_num =0;
    //std::vector<IMUDATA>::iterator it = imu_data.begin();
    for (const rosbag::MessageInstance& m : view_pic){
        if (!ros::ok()){
            break;
        }
        /*if (decframe){
            decframe = false;
        }
        else
        {
            decframe = true;
        }

        if (decframe){
            continue;
        }*/
        sensor_msgs::Image::Ptr img = m.instantiate<sensor_msgs::Image>();
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(img);
        }catch (cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            continue;
        }
        cv::Mat img_0 = cv_ptr->image.clone();
        double timep = (*img).header.stamp.toSec();
        ROS_INFO("pictime is %f", timep);
        //weizi
        Eigen::Matrix4d T_M_I = Eigen::Matrix4d::Identity();
        T_M_I.block(0,0,3,3)=gt_all.at(gt_num).rotation;
        T_M_I.block(0,3,3,1)=gt_all.at(gt_num).transpose;
        //update
        vio::VioUpdate update;
        update.timestamp_ns = (int64_t)(1e9*timep);
        update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_M_I));
        aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
        std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
        frame->setCameraGeometry(camera_rig->getCameraShared(0));
        frame->setId(aslam::FrameId::Random());
        frame->setTimestampNanoseconds((int64_t)(1e9*timep));
        frame->clearKeypointChannels();
        frame->setRawImage(img_0.clone());
        nframe->setFrame(0, frame);
        vio::SynchronizedNFrameImu sysnc_nframe;
        sysnc_nframe.nframe = nframe;
        sysnc_nframe.motion_wrt_last_nframe = vio::MotionType::kGeneralMotion;
        if (last_timestamp == -1){
            update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sysnc_nframe);
            builder->apply(update);
            map->storeRawImage(sysnc_nframe.nframe->getFrame(0u).getRawImage(), 0u, map->getVertexPtr(builder->getLastVertexId()));
            last_timestamp = timep;
            continue;
        }
        std::vector<test::IMUDATA> imu;
        for(size_t i = 0; i < imu_data.size()-1; i++){
            if (imu_data.at(i).timestamp <= last_timestamp && imu_data.at(i+1).timestamp > last_timestamp){
                test::IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), last_timestamp);
                //ROS_INFO("imu time is :%f", data.timestamp);
                imu.push_back(data);
                continue;
            }
            if (imu_data.at(i).timestamp > last_timestamp && imu_data.at(i+1).timestamp < timep){
                imu.push_back(imu_data.at(i));
                //ROS_INFO("imu time is :%f", imu_data.at(i).timestamp);
                continue;
            }
            if (imu_data.at(i).timestamp <= timep && imu_data.at(i+1).timestamp > timep){
                imu.push_back(imu_data.at(i));
                if (imu.at(imu.size()-1).timestamp != timep){
                    test::IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), timep);
                    imu.push_back(data);
                }
                ROS_INFO("last imu time is %f", imu.at(imu.size()-1).timestamp);
                break;
            }

        }
        ROS_INFO("imu num is %d", imu.size());
        if (imu.empty() ){
            ROS_ERROR("There are not enough imu measurements");
            continue;
        }

        sysnc_nframe.imu_timestamps.resize(1, imu.size());
        sysnc_nframe.imu_measurements.resize(6, imu.size());
        for (size_t i=0; i < imu.size(); i++){
            sysnc_nframe.imu_timestamps(i) = (int64_t)(1e9*imu.at(i).timestamp);
            sysnc_nframe.imu_measurements.block(0,i,3,1) = imu.at(i).am;
            sysnc_nframe.imu_measurements.block(3,i,3,1) = imu.at(i).wm;
        }
        update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sysnc_nframe);
        builder->apply(update);
        map->storeRawImage(sysnc_nframe.nframe->getFrame(0u).getRawImage(), 0u, map->getVertexPtr(builder->getLastVertexId()));
        last_timestamp = timep;
        gt_num++;

    }
    feature_tracking::VOFeatureTrackingPipeline trackpipe;
    trackpipe.runTrackingAndTriangulationForAllMissions(map);
    builder->checkConsistency();

    backend::SaveConfig save_config;
    save_config.overwrite_existing_files = true;
    vi_map::serialization::saveMapToFolder(map_folder, save_config, map);
    ROS_INFO("succeed");
    delete (map);
    delete(builder);
    return 0;
}


std::shared_ptr<aslam::NCamera> get_camera_rig(ros::NodeHandle n){
    std::shared_ptr<aslam::NCamera> camera_rig;
    std::vector<int> resolution;
    Eigen::Matrix<double, 4, 1> cam_proj;
    Eigen::Matrix<double, 4, 1> cam_dist;
    Eigen::Matrix4d T_I_C = Eigen::Matrix4d::Identity();
    test::camera_config_from_yaml(n,cam_proj, cam_dist, T_I_C, resolution);
    std::vector<std::shared_ptr<aslam::Camera>> cameras;
    bool is_fisheye;
    n.param<bool>("is_fisheye", is_fisheye, true);
    aslam::Distortion* distptr;
    if(is_fisheye){
        distptr = new aslam::EquidistantDistortion(cam_dist);
    } else{
        distptr = new aslam::RadTanDistortion(cam_dist);
    }
    aslam::Distortion::UniquePtr uniqdistptr(distptr);
    aslam::Camera::Ptr camera = std::make_shared<aslam::PinholeCamera>(cam_proj, resolution.at(0),
                                                                       resolution.at(1), uniqdistptr);
    camera->setId(aslam::CameraId::Random());
    cameras.push_back(camera);
    aslam::TransformationVector T_C_Ii;
    Eigen::Matrix4d T_C_I = T_I_C.inverse();
    T_C_Ii.push_back(aslam::Transformation::constructAndRenormalizeRotation(T_C_I));
    std::string cam_lable = "ncamera";
    camera_rig = std::make_shared<aslam::NCamera>(aslam::NCameraId::Random(), T_C_Ii, cameras, cam_lable);
    return camera_rig;
}




