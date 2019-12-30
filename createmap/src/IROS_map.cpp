//
// Created by zzh on 2019/12/11.
//
#include <ros/ros.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
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
#include <IROS_map.h>

std::map<double, IROS::POSE> gt_data;
std::vector<IROS::IMUDATA> imu_data;
std::vector<IROS::PIC> pic_data;
std::vector<IROS::ACC> acc_data;
std::vector<IROS::GYR> gyr_data;
//vi_map::VIMap* map;
//online_map_builders::StreamMapBuilder* builder;
//std::shared_ptr<aslam::NCamera> camera_rig;
size_t ite=5;
void acc_callback(const sensor_msgs::Imu::ConstPtr& acc_msg);
void gyr_callback(const sensor_msgs::Imu::ConstPtr& gyr_msg);
void img_callback(const sensor_msgs::Image::ConstPtr& img_msg);
void gt_callback(const nav_msgs::Odometry::ConstPtr& gt);
std::shared_ptr<aslam::NCamera> get_camera_rig(ros::NodeHandle n);
int main(int argc, char ** argv){
    ros::init(argc, argv, "IROS_map");
    ros::NodeHandle n("~");
    vi_map::ImuSigmas imu_sigma;
    IROS::imuconfig_from_yaml(n, imu_sigma);
    //imu_config
    constexpr char KImuHardwareID[] = "imu0";
    vi_map::SensorId imu_sensor_Id;
    common::generateId(&imu_sensor_Id);
    vi_map::Imu::UniquePtr imu_rig = aligned_unique<vi_map::Imu>(imu_sensor_Id, static_cast<std::string>(KImuHardwareID));
    imu_rig->setImuSigmas(imu_sigma);
    //camera_config
    std::shared_ptr<aslam::NCamera> camera_rig = get_camera_rig(n);

    std::string cam_topic;
    std::string acc_topic;
    std::string gyr_topic;
    std::string gt_topic;
    std::string map_folder;
    IROS::get_topic_and_path(n, cam_topic, acc_topic, gyr_topic, gt_topic, map_folder);

    vi_map::VIMap* map = new vi_map::VIMap(map_folder);
    online_map_builders::StreamMapBuilder* builder = new online_map_builders::StreamMapBuilder(camera_rig, std::move(imu_rig), map);
    double last_timestamp = -1;
    ros::Subscriber pic = n.subscribe(cam_topic, 100, img_callback);
    ros::Subscriber acc = n.subscribe(acc_topic, 100, acc_callback);
    ros::Subscriber gyr = n.subscribe(gyr_topic, 100, gyr_callback);
    ros::Subscriber gt = n.subscribe(gt_topic, 100, gt_callback);
    ros::Rate loop_rate(5);
    while (ros::ok()){
        //std::cout << "yunxing" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    for (size_t i=0; i < acc_data.size(); i++){
        if (acc_data.at(i).timestamp < gyr_data.front().timestamp){
            continue;
        }
        double acc_time = acc_data.at(i).timestamp;
        IROS::GYR gyr_inter;
        for (size_t j=0; j < gyr_data.size()-1; j++){
            if (gyr_data.at(j).timestamp <= acc_time && gyr_data.at(j+1).timestamp > acc_time){
                gyr_inter = IROS::inter_gyr(gyr_data.at(j), gyr_data.at(j+1), acc_time);
                break;
            }
        }
        IROS::IMUDATA imu;
        imu.timestamp = acc_time;
        imu.am = acc_data.at(i).am;
        imu.wm = gyr_inter.wm;
        imu_data.push_back(imu);
    }

    std::cout << "imu size is" << imu_data.size() << std::endl;
    for (int i=0; i < pic_data.size(); i ++){
        IROS::PIC cur_frame = pic_data.at(i);
        double cur_timestamp = cur_frame.timestamp;
        Eigen::Matrix4d T_M_I = Eigen::Matrix4d::Identity();
        if (gt_data.count(cur_timestamp)){
            T_M_I.block(0,0,3,3)=gt_data.find(cur_timestamp)->second.rotation;
            T_M_I.block(0,3,3,1)=gt_data.find(cur_timestamp)->second.transpose;
        } else{
            LOG(ERROR) << "could not find gt";
            continue;
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
        std::vector<IROS::IMUDATA> imu;
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
                IROS::IMUDATA data = interpolate_data(imu_data.at(ite), imu_data.at(ite+1), last_timestamp);
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
                    IROS::IMUDATA data = interpolate_data(imu_data.at(ite), imu_data.at(ite+1), cur_timestamp);
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
    return 0;
}


std::shared_ptr<aslam::NCamera> get_camera_rig(ros::NodeHandle n){
    std::shared_ptr<aslam::NCamera> camera_rig;
    std::vector<int> resolution;
    Eigen::Matrix<double, 4, 1> cam_proj;
    Eigen::Matrix<double, 4, 1> cam_dist;
    Eigen::Matrix4d T_I_C = Eigen::Matrix4d::Identity();
    IROS::camera_config_from_yaml(n,cam_proj, cam_dist, T_I_C, resolution);
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



void acc_callback(const sensor_msgs::Imu::ConstPtr& acc_msg){
    double timestamp = acc_msg->header.stamp.toSec();
    Eigen::Matrix<double, 3,1> am;
    am << acc_msg->linear_acceleration.x, acc_msg->linear_acceleration.y, acc_msg->linear_acceleration.z;
    IROS::ACC data;
    data.timestamp = timestamp;
    data.am = am;
    acc_data.push_back(data);
}
void gyr_callback(const sensor_msgs::Imu::ConstPtr& gyr_msg){
    double timestamp = gyr_msg->header.stamp.toSec();
    Eigen::Vector3d wm;
    wm << gyr_msg->angular_velocity.x, gyr_msg->angular_velocity.y, gyr_msg->angular_velocity.z;
    IROS::GYR data;
    data.timestamp = timestamp;
    data.wm = wm;
    gyr_data.push_back(data);
}
void img_callback(const sensor_msgs::Image::ConstPtr& img_msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }catch (cv_bridge::Exception &e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    IROS::PIC img1;
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
    IROS::POSE data;
    //data.timestamp = timestamp;
    data.rotation = q.toRotationMatrix();
    data.transpose = t;
    gt_data.insert(std::make_pair(timestamp, data));
}