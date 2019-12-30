//
// Created by zzh on 2019/12/11.
//
#include <ros/ros.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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
#include <glog/logging.h>
#include <exception>
std::vector<mapbuilder::IMUDATA> imu_data;
std::map<double, mapbuilder::pose> gt_data;
std::vector<mapbuilder::pic_frame> pic_data;

//vi_map::VIMap* map;
//online_map_builders::StreamMapBuilder* builder;
//std::shared_ptr<aslam::NCamera> camera_rig;
size_t ite=5;
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
void img_callback(const sensor_msgs::Image::ConstPtr& img_msg);
void gt_callback(const nav_msgs::Odometry::ConstPtr& gt);
std::shared_ptr<aslam::NCamera> get_camera_rig(ros::NodeHandle n);
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
    imu_rig->setImuSigmas(imu_sigma);
    //camera_config
    std::shared_ptr<aslam::NCamera> camera_rig = get_camera_rig(n);

    std::string cam_topic;
    std::string imu_topic;
    std::string gt_topic;
    std::string map_folder;
    mapbuilder::get_topic_and_path(n, cam_topic, imu_topic, gt_topic, map_folder);

    vi_map::VIMap* map = new vi_map::VIMap(map_folder);
    online_map_builders::StreamMapBuilder* builder = new online_map_builders::StreamMapBuilder(camera_rig, std::move(imu_rig), map);
    double last_timestamp = -1;
    ros::Subscriber pic = n.subscribe(cam_topic, 100, img_callback);
    ros::Subscriber imu = n.subscribe(imu_topic, 100, imu_callback);
    ros::Subscriber gt = n.subscribe(gt_topic, 100, gt_callback);
    ros::Rate loop_rate(5);
    while (ros::ok()){
        //std::cout << "yunxing" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    for (int i=0; i < pic_data.size(); i ++){
        mapbuilder::pic_frame cur_frame = pic_data.at(i);
        double cur_timestamp = cur_frame.timestamp;
        Eigen::Matrix4d T_M_I = Eigen::Matrix4d::Identity();
        if (gt_data.count(cur_timestamp)){
            T_M_I.block(0,0,3,3)=gt_data.find(cur_timestamp)->second.rotation;
            T_M_I.block(0,3,3,1)=gt_data.find(cur_timestamp)->second.transpose;
        } else{

            LOG(ERROR) << "could not find gt , so interplate";

            //continue;
            auto gt_after = gt_data.lower_bound(cur_timestamp);
            double time_after = gt_after->first;
            mapbuilder::pose pose_after = gt_after->second;
            if (gt_after == gt_data.begin()){
                continue;
            }
            try {
                gt_after--;
                double time_befor = gt_after->first;
                mapbuilder::pose pose_before = gt_after->second;

                mapbuilder::pose cur_pose = mapbuilder::interplate_gt(pose_before, time_befor, pose_after, time_after,
                                                                      cur_timestamp);
                T_M_I.block(0, 0, 3, 3) = cur_pose.rotation;
                T_M_I.block(0, 3, 3, 1) = cur_pose.transpose;
            }
            catch (std::exception &e){
                std::cout << e.what() << std::endl;
                continue;
            }
        }
        vio::VioUpdate update;
        update.timestamp_ns = (int64_t)(1e9*cur_timestamp);
        update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_M_I));
        aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
        std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
        frame->setCameraGeometry(camera_rig->getCameraShared(0));
        frame->setId(aslam::FrameId::Random());
        frame->setTimestampNanoseconds((int64_t)(1e9*cur_timestamp));
        frame->clearKeypointChannels();
        frame->setRawImage(cur_frame.image.clone());
        nframe->setFrame(0,frame);
        vio::SynchronizedNFrameImu sysnc_nframe;
        sysnc_nframe.nframe = nframe;
        sysnc_nframe.motion_wrt_last_nframe = vio::MotionType::kGeneralMotion;
        if ( last_timestamp == -1){
            update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sysnc_nframe);
            builder->apply(update);
            map->storeRawImage(sysnc_nframe.nframe->getFrame(0u).getRawImage(), 0u, map->getVertexPtr(builder->getLastVertexId()));
            last_timestamp = cur_timestamp;
            continue;
        }
        std::vector<mapbuilder::IMUDATA> imu;
        /*for (size_t i=0; i < imu_data.size()-1; i++){
            if (imu_data.at(i).timestamp <= last_timestamp && imu_data.at(i+1).timestamp > last_timestamp){
                mapbuilder::IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), last_timestamp);
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
                    mapbuilder::IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), cur_timestamp);
                    imu.push_back(data);
                }
                ROS_INFO("last imu time is %f", imu.at(imu.size()-1).timestamp);
                LOG(INFO) << "last imu time is " << imu.at(imu.size()-1).timestamp;
                break;
            }

        }*/
        for (ite=ite-5; ite < imu_data.size()-1; ite++){
            if (imu_data.at(ite).timestamp <= last_timestamp && imu_data.at(ite+1).timestamp > last_timestamp){
                mapbuilder::IMUDATA data = interpolate_data(imu_data.at(ite), imu_data.at(ite+1), last_timestamp);
                //ROS_INFO("imu time is :%f", data.timestamp);
                imu.push_back(data);
                continue;
            }
            if (imu_data.at(ite).timestamp > last_timestamp && imu_data.at(ite+1).timestamp < cur_timestamp){
                imu.push_back(imu_data.at(ite));
                //ROS_INFO("imu time is :%f", imu_data.at(ite).timestamp);
                continue;
            }
            if (imu_data.at(ite).timestamp <= cur_timestamp && imu_data.at(ite+1).timestamp > cur_timestamp){
                imu.push_back(imu_data.at(ite));
                if (imu.at(imu.size()-1).timestamp != cur_timestamp){
                    mapbuilder::IMUDATA data = interpolate_data(imu_data.at(ite), imu_data.at(ite+1), cur_timestamp);
                    imu.push_back(data);
                }


                break;
            }

        }
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
        last_timestamp = cur_timestamp;
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
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }catch (cv_bridge::Exception &e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mapbuilder::pic_frame img1;
    img1.timestamp = img_msg->header.stamp.toSec();
    ROS_INFO("time is %f", img1.timestamp);
    img1.image = cv_ptr->image.clone();
    pic_data.push_back(img1);
}
void gt_callback(const nav_msgs::Odometry::ConstPtr& gt){
    double timestamp = gt->header.stamp.toSec();
    Eigen::Quaterniond q(gt->pose.pose.orientation.w, gt->pose.pose.orientation.x,
                         gt->pose.pose.orientation.y, gt->pose.pose.orientation.z);
    Eigen::Vector3d t(gt->pose.pose.position.x, gt->pose.pose.position.y, gt->pose.pose.position.z);
    mapbuilder::pose data;
    //data.timestamp = timestamp;
    data.rotation = q.toRotationMatrix();
    data.transpose = t;
    gt_data.insert(std::make_pair(timestamp, data));
}