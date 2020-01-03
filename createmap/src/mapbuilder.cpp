//
// Created by zzh on 2019/12/11. 读取话题过程中处理
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
#include <mapbuilder.h>
#include <mutex>

std::vector<mapbuilder::IMUDATA> imu_data;
std::map<double, mapbuilder::pose> gt_data;
std::vector<mapbuilder::pic_frame> pic_data;
vi_map::VIMap* map;
online_map_builders::StreamMapBuilder* builder;
std::shared_ptr<aslam::NCamera> camera_rig;
double last_timestamp = -1;
double cur_timestamp = -1;
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
void img_callback(const sensor_msgs::Image::ConstPtr& img_msg);
void gt_callback(const geometry_msgs::PoseStamped::ConstPtr& gt);
std::shared_ptr<aslam::NCamera> get_camera_rig(ros::NodeHandle n);
std::mutex mu;

int main(int argc, char ** argv){
    ros::init(argc, argv, "mapbuilder");
    ros::NodeHandle n("~");
    vi_map::ImuSigmas imu_sigma;
    mapbuilder::imuconfig_from_yaml(n, imu_sigma);
    //imu_config
    constexpr char KImuHardwareID[] = "imu0";
    vi_map::SensorId imu_sensor_Id;
    common::generateId(&imu_sensor_Id);
    vi_map::Imu::UniquePtr imu_rig = aligned_unique<vi_map::Imu>(imu_sensor_Id, static_cast<std::string>(KImuHardwareID));
    imu_rig->setImuSigmas(imu_sigma);//IMU配置
    camera_rig = get_camera_rig(n); //相机配置

    std::string cam_topic;
    std::string imu_topic;
    std::string gt_topic;
    std::string map_folder;
    mapbuilder::get_topic_and_path(n, cam_topic, imu_topic, gt_topic, map_folder);

    map = new vi_map::VIMap(map_folder);
    builder = new online_map_builders::StreamMapBuilder(camera_rig, std::move(imu_rig), map);

    ros::Subscriber pic = n.subscribe(cam_topic, 100, img_callback);
    ros::Subscriber imu = n.subscribe(imu_topic, 100, imu_callback);
    ros::Subscriber gt = n.subscribe(gt_topic, 100, gt_callback);

    ros::spin();
    feature_tracking::VOFeatureTrackingPipeline trackpipe;
    trackpipe.runTrackingAndTriangulationForAllMissions(map);
    builder->checkConsistency();
    /*backend::SaveConfig save_config;
    save_config.overwrite_existing_files = true;
    vi_map::serialization::saveMapToFolder(map_folder, save_config, map);
    std::cout << "succeed" << std::endl;*/
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
    mapbuilder::camera_config_from_yaml(n,cam_proj, cam_dist, T_I_C, resolution);
    std::vector<std::shared_ptr<aslam::Camera>> cameras;
    bool is_fisheye;
    n.param<bool>("is_fisheye", is_fisheye, true);
    aslam::Distortion* distptr;
    if(is_fisheye){
        distptr = new aslam::EquidistantDistortion(cam_dist);
    } else{
        distptr = new aslam::RadTanDistortion(cam_dist);
    }
    aslam::Distortion::UniquePtr uniqdistptr(distptr); //畸变模型
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



void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    double timestamp = imu_msg->header.stamp.toSec();
    Eigen::Matrix<double, 3,1> am, wm;
    wm << imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z;
    am << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
    mapbuilder::IMUDATA data;
    data.timestamp = timestamp;
    data.am = am;
    data.wm = wm;
    imu_data.push_back(data);
}
void img_callback(const sensor_msgs::Image::ConstPtr& img_msg){
    if (gt_data.empty()){ //等待位姿
        ROS_ERROR("there is no pose estimation");
        imu_data.clear();

        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }catch (cv_bridge::Exception e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img_1 = cv_ptr->image.clone();
    double timestamp = img_msg->header.stamp.toSec();
    ROS_INFO("picccc time is %f", cur_timestamp);
    mapbuilder::pic_frame data;
    data.timestamp = timestamp;
    data.image = img_1.clone();
    Eigen::Matrix4d T_M_I = Eigen::Matrix4d::Identity();
    vio::VioUpdate update;
    if (last_timestamp == -1){ //第一帧图片
        pic_data.push_back(data);
        last_timestamp = timestamp;
        return;
    }
    if (cur_timestamp == -1){ //第二帧图片，处理第一帧图片
        mu.lock();
        if (gt_data.count(last_timestamp)){
            T_M_I.block(0,0,3,3)=gt_data.find(last_timestamp)->second.rotation;
            T_M_I.block(0,3,3,1)=gt_data.find(last_timestamp)->second.transpose;
        }
        else {
            ROS_ERROR("could not get pose estimation ");
        }
        mu.unlock();
        update.timestamp_ns = (int64_t)(1e9*last_timestamp);
        update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_M_I));
        std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
        aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
        frame->setCameraGeometry(camera_rig->getCameraShared(0));
        frame->setId(aslam::FrameId::Random());
        frame->setTimestampNanoseconds((int64_t)(1e9*last_timestamp));
        frame->clearKeypointChannels();
        frame->setRawImage(pic_data.back().image.clone());
        nframe->setFrame(0, frame);
        vio::SynchronizedNFrameImu sysnc_nframe;
        sysnc_nframe.nframe = nframe;
        sysnc_nframe.motion_wrt_last_nframe = vio::MotionType::kGeneralMotion;
        update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sysnc_nframe);
        builder->apply(update);
        map->storeRawImage(sysnc_nframe.nframe->getFrame(0u).getRawImage(), 0u, map->getVertexPtr(builder->getLastVertexId()));
        pic_data.push_back(data);
        cur_timestamp = timestamp;
        return;
    }
    mu.lock();
    //后续帧，处理前一帧
    if (gt_data.count(cur_timestamp)){
        T_M_I.block(0,0,3,3)=gt_data.find(cur_timestamp)->second.rotation;
        T_M_I.block(0,3,3,1)=gt_data.find(cur_timestamp)->second.transpose;
    }
    else {
        ROS_ERROR("could not get pose estimation ");
    }
    mu.unlock();
    update.timestamp_ns = (int64_t)(1e9*cur_timestamp);
    update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_M_I));
    std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
    aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
    frame->setCameraGeometry(camera_rig->getCameraShared(0));
    frame->setId(aslam::FrameId::Random());
    frame->setTimestampNanoseconds((int64_t)(1e9*cur_timestamp));
    frame->clearKeypointChannels();
    frame->setRawImage(pic_data.back().image.clone());
    nframe->setFrame(0, frame);
    vio::SynchronizedNFrameImu sysnc_nframe;
    sysnc_nframe.nframe = nframe;
    sysnc_nframe.motion_wrt_last_nframe = vio::MotionType::kGeneralMotion;

    std::vector<mapbuilder::IMUDATA> imu;
    /*for(size_t i = 0; i < imu_data.size()-1; i++){
        if (imu_data.at(i).timestamp <= last_timestamp && imu_data.at(i+1).timestamp > last_timestamp){
            mapbuilder::IMUDATA data = mapbuilder::interpolate_data(imu_data.at(i), imu_data.at(i+1), last_timestamp);
            //ROS_INFO("imu time is :%f", data.timestamp);
            imu.push_back(data);
            continue;
        }
        if (imu_data.at(i).timestamp > last_timestamp && imu_data.at(i+1).timestamp < cur_timestamp){
            imu.push_back(imu_data.at(i));
            //ROS_INFO("imu time is :%f", imu_data.at(i).timestamp);
            continue;
        }
        if (imu_data.at(i).timestamp <= cur_timestamp && imu_data.at(i+1).timestamp > cur_timestamp){
            imu.push_back(imu_data.at(i));
            if (imu.at(imu.size()-1).timestamp != cur_timestamp){
                mapbuilder::IMUDATA data = mapbuilder::interpolate_data(imu_data.at(i), imu_data.at(i+1), cur_timestamp);
                imu.push_back(data);
            }
            ROS_INFO("last imu time is %f", imu.at(imu.size()-1).timestamp);
            ROS_INFO("imu num is %d", imu.size());
            break;
        }

    }*/
    //从后向前遍历IMU找到帧与帧之间的IMU数据，同时插值出帧对应的IMU数据
    mu.lock();
    for (size_t i=imu_data.size()-1; i >0; i--){
        if (imu_data.at(i).timestamp >=cur_timestamp && imu_data.at(i-1).timestamp <cur_timestamp){
            mapbuilder::IMUDATA data = mapbuilder::interpolate_data(imu_data.at(i-1), imu_data.at(i), cur_timestamp);
            imu.push_back(data);
            continue;
        }
        if (imu_data.at(i).timestamp < cur_timestamp && imu_data.at(i-1).timestamp >last_timestamp){
            imu.push_back(imu_data.at(i));
            continue;
        }
        if (imu_data.at(i).timestamp >=last_timestamp && imu_data.at(i-1).timestamp < last_timestamp){
            imu.push_back(imu_data.at(i));
            if (imu.at(imu.size()-1).timestamp !=last_timestamp){
                mapbuilder::IMUDATA data = mapbuilder::interpolate_data(imu_data.at(i-1), imu_data.at(i), last_timestamp);
                imu.push_back(data);
            }
            break;
        }
    }
    mu.unlock();
    if (imu.empty() ){
        ROS_ERROR("There are not enough imu measurements");
        return;
    }
    sysnc_nframe.imu_timestamps.resize(1, imu.size());
    sysnc_nframe.imu_measurements.resize(6, imu.size());
    /*for (size_t i=0; i < imu.size(); i++){
        sysnc_nframe.imu_timestamps(i) = (int64_t)(1e9*imu.at(i).timestamp);
        sysnc_nframe.imu_measurements.block(0,i,3,1) = imu.at(i).am;
        sysnc_nframe.imu_measurements.block(3,i,3,1) = imu.at(i).wm;
    }*/
    ROS_INFO("IMU size %d", imu.size());
    for (int i=imu.size()-1; i >=0; i--){
        int j = imu.size() -1 - i;
        sysnc_nframe.imu_timestamps(j) = (int64_t)(1e9*imu.at(i).timestamp);
        sysnc_nframe.imu_measurements.block(0,j,3,1) = imu.at(i).am;
        sysnc_nframe.imu_measurements.block(3,j,3,1)=imu.at(i).wm;
    }
    update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sysnc_nframe);
    ROS_INFO("imu first time is %f", imu.back().timestamp);
    ROS_INFO("imu last time is %f", imu.front().timestamp);
    builder->apply(update);
    map->storeRawImage(sysnc_nframe.nframe->getFrame(0u).getRawImage(), 0u, map->getVertexPtr(builder->getLastVertexId()));

    last_timestamp = cur_timestamp;
    cur_timestamp = timestamp;
    pic_data.push_back(data);
}
void gt_callback(const geometry_msgs::PoseStamped::ConstPtr& gt){
    double timestamp = gt->header.stamp.toSec();
    Eigen::Quaterniond q(gt->pose.orientation.w, gt->pose.orientation.x,
                         gt->pose.orientation.y, gt->pose.orientation.z);
    Eigen::Vector3d t(gt->pose.position.x, gt->pose.position.y, gt->pose.position.z);
    mapbuilder::pose data;
    //data.timestamp = timestamp;
    data.rotation = q.toRotationMatrix();
    data.transpose = t;
    gt_data.insert(std::make_pair(timestamp, data));
}