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
#include <vio-common/vio-update.h>
#include <online-map-builders/stream-map-builder.h>
#include <feature-tracking/vo-feature-tracking-pipeline.h>
#include <vi-map/vi-map-serialization.h>
 struct IMUDATA {

            /// Timestamp of the reading
            double timestamp;

            /// Gyroscope reading, angular velocity (rad/s)
            Eigen::Matrix<double, 3, 1> wm;

            /// Accelerometer reading, linear acceleration (m/s^2)
            Eigen::Matrix<double, 3, 1> am;

        };
struct groudtruth{
    double timestamp;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d transpose;
};
IMUDATA interpolate_data(const IMUDATA imu_1, const IMUDATA imu_2, double timestamp);
/*std::vector<IMUDATA> imu_data;
int count =0;
void imu_subcallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
     double time_stamp = imu_msg->header.stamp.toSec();
     Eigen::Matrix<double, 3,1> am, wm;
     am << imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z;
     wm << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
     IMUDATA data;
     data.timestamp = time_stamp;
     data.am = am;
     data.wm = wm;
     imu_data.push_back(data);
     
}*/
//std::shared_ptr<aslam::NCamera> came_rig( ros::NodeHandle nh);
int main (int argc, char** argv){
    ros::init(argc, argv, "mapbuilder");
    ros::NodeHandle nh;
    //ROS_INFO("maxsize: %d", imu_data.size());
    /*ros::Subscriber imu_sub = nh.subscribe("/camera/imu", 1000, imu_subcallback);
    ros::spin();*/
    //相机内参
    std::vector<std::shared_ptr<aslam::Camera>> cameras;
    std::vector<int> resolution = {640,480};
    std::vector<double> matrix_k_default = {260.628,259.046,311.885,234.024};
    std::vector<double> matrix_d_default = {0.009021128160543755,-0.02923106982675979,0.03575099139197932,-0.01305653026438099};
    Eigen::Matrix<double, 4, 1> cam_proj;
    Eigen::Matrix<double, 4, 1> cam_dist;
    cam_proj << matrix_k_default.at(0), matrix_k_default.at(1), matrix_k_default.at(2), matrix_k_default.at(3);
    cam_dist << matrix_d_default.at(0), matrix_d_default.at(1), matrix_d_default.at(2), matrix_d_default.at(3);
    bool is_fisheye = true;
    aslam::Distortion* distptr;
    if(is_fisheye) {
            distptr = new aslam::EquidistantDistortion(cam_dist);
        } else{
            distptr = new aslam::RadTanDistortion(cam_dist);
    }
    aslam::Distortion::UniquePtr uniqdistptr(distptr);
    std::shared_ptr<aslam::Camera> camera= std::make_shared<aslam::PinholeCamera>(cam_proj, resolution.at(0), resolution.at(1), uniqdistptr);
    camera->setId(aslam::CameraId::Random());
    cameras.push_back(camera);
    //相机外参
    aslam::TransformationVector T_C_Ii;
    Eigen::Matrix4d T_I_C;
    T_I_C << 0.9999124458318692,0.013232486795308555,4.4314123813978e-05,0.0068797026485456735,
             -0.013231413348487249,0.9997760129775242,0.016518280042641727,0.0021988518878476603,
             0.00017427372452013417,-0.016517420136862998, 0.9998635629228078, -3.6631173134439025e-05,
             0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4d T_C_I = T_I_C.inverse();
    T_C_Ii.push_back( aslam::Transformation::constructAndRenormalizeRotation(T_C_I));
    //相机对象
    std::shared_ptr<aslam::NCamera> camera_rig;
    std::string cam_lable = "ncameras";
    camera_rig = std::make_shared<aslam::NCamera>(aslam::NCameraId::Random(), T_C_Ii, cameras, cam_lable);
    
    
    //imu_sigma
    vi_map::ImuSigmas imu_sigma;
    imu_sigma.acc_bias_random_walk_noise_density=9.9999997473787516e-05;
    imu_sigma.acc_noise_density=6.6952452471014112e-05;
    imu_sigma.gyro_bias_random_walk_noise_density=4.9999999873762135e-07;
    imu_sigma.gyro_noise_density=5.1480301408446394e-06;
    constexpr char KImuHardwareId[]="imu0";
    vi_map::SensorId imu_sensor_id;
    common::generateId(&imu_sensor_id);
    vi_map::Imu::UniquePtr imu_sensor = aligned_unique<vi_map::Imu>(imu_sensor_id, static_cast<std::string>(KImuHardwareId));
    imu_sensor->setImuSigmas(imu_sigma);
    std::string path_to_map = "/home/zzh/sharetohost/rosbag/";
    //地图
    vi_map::VIMap * map = new vi_map::VIMap(path_to_map);
    online_map_builders::StreamMapBuilder* builder = new online_map_builders::StreamMapBuilder(camera_rig, std::move(imu_sensor), map);
    
    
    //std::string path_to_bag = "/home/zzh/sharetohost/rosbag/cafe1.bag";
    std::string path_to_bag = "/home/zzh/sharetohost/rosbag/111.bag";
    std::string path_to_gt = "/home/zzh/sharetodocker/rosbag/groundtruth";
    

    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);
    //std::string topic_imu = "/camera/imu";
    std::string topic_imu = "/imu0";
    //std::string topic_pic = "/camera/fisheye1/image_raw";
    std::string topic_pic = "/cam0/image_raw";
    std::string topic_gt = "/tf0";
    rosbag::View view_imu(bag, rosbag::TopicQuery(topic_imu));
    rosbag::View view_pic(bag, rosbag::TopicQuery(topic_pic));
    rosbag::View view_gt(bag, rosbag::TopicQuery(topic_gt));
    std::vector<IMUDATA> imu_data;
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
        IMUDATA data;
        data.timestamp = timem;
        data.am = am;
        data.wm = wm;
        imu_data.push_back(data);
        i ++ ;
    }
    ROS_INFO("num is %d", imu_data.size());

    std::vector<groudtruth> gt_all;
    for (const rosbag::MessageInstance & m : view_gt){
        if (!ros::ok()){
            break;
        }
        geometry_msgs::PoseStamped::ConstPtr gt = m.instantiate<geometry_msgs::PoseStamped>();
        double timeg = (*gt).header.stamp.toSec();
        Eigen::Quaterniond q((*gt).pose.orientation.w, (*gt).pose.orientation.x, (*gt).pose.orientation.y, (*gt).pose.orientation.z);
        Eigen::Vector3d t;
        t << (*gt).pose.position.x, (*gt).pose.position.y, (*gt).pose.position.z;
        groudtruth gt_1;
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
        std::vector<IMUDATA> imu;
        for(size_t i = 0; i < imu_data.size()-1; i++){
            if (imu_data.at(i).timestamp <= last_timestamp && imu_data.at(i+1).timestamp > last_timestamp){
                IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), last_timestamp);
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
                    IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), timep);
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
    vi_map::serialization::saveMapToFolder(path_to_map, save_config, map);
    ROS_INFO("succeed");
    return 0;

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
/*std::shared_ptr<aslam::NCamera> camrig( ros::NodeHandle nh){
    //相机内参
    std::vector<std::shared_ptr<aslam::Camera>> cameras;
    std::vector<int> resolution = {752,480};
    std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
    std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};
    Eigen::Matrix<double, 4, 1> cam_proj;
    Eigen::Matrix<double, 4, 1> cam_dist;
    cam_proj << matrix_k_default.at(0), matrix_k_default.at(1), matrix_k_default.at(2), matrix_k_default.at(3);
    cam_proj << matrix_d_default.at(0), matrix_d_default.at(1), matrix_d_default.at(2), matrix_d_default.at(3);
    bool is_fisheye;
    aslam::Distortion* distptr;
    if(is_fisheye) {
            distptr = new aslam::EquidistantDistortion(cam_dist);
        } else{
            distptr = new aslam::RadTanDistortion(cam_dist);
    }
    aslam::Distortion::UniquePtr uniqdist(distptr);
    std::shared_ptr<aslam::Camera> camera = std::make_shared<aslam::PinholeCamera>(cam_proj, resolution.at(0), resolution.at(1), uniqdist);
    cameras.push_back(camera);
    //相机外参
    Eigen::Matrix4d T_I_C;
    T_I_C << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    Eigen::Matrix4d T_C_I = T_I_C.inverse();
    std::shared_ptr<aslam::NCamera> camera_rig;
    std::string cam_lable = "ncameras";
    camera_rig = std::make_shared<aslam::NCamera>(aslam::NCameraId::Random(), T_C_I, cameras, cam_lable);
    return came_rig;

}*/