/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "pcl_ros/impl/transforms.hpp"

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>

#include <pclomp/ndt_omp.h>

// #include <runtime_manager/ConfigNdtMapping.h>
// #include <runtime_manager/ConfigNdtMappingOutput.h>

// #include <pointshape_processor.h>

struct Position {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};
using namespace std;

// global variables
static Position previous_pos, guess_pos, current_pos, added_pos;

static double offset_x, offset_y, offset_z, offset_yaw;  // current_pos - previous_pos

static pcl::PointCloud<pcl::PointXYZI> map_global;
ros::Timer timer;

/////////////////////// Xi  ///////////////////////////////////
std::vector<pcl::PointCloud<pcl::PointXYZI>> map_local;      // for ndt registration
std::vector<pcl::PointCloud<pcl::PointXYZRGB>> map_terrain;  // for publish

// Pointshape_Processor ps_processor(360*4);

int map_local_index = 0;
int map_local_length = 5;

int map_terrain_index = 0;
int map_terrain_length = 50;
float shift_terrain = 0.05;

tf::TransformListener *tfListener = NULL;
//////////////////////////////////////////////////////////////////////

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int iter = 30;            // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.5;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 1.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
        t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher local_map_pub;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg;

static ros::Publisher ndt_stat_pub, pub_velodyne_base;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double RANGE = 0.0;
static double SHIFT = 0.0;

tf::Transform g_transform;
double roll, pitch, yaw;
double first_roll, first_pitch, first_yaw;
bool is_FirstImu = true;
/////////////////////// Xi  ///////////////////////////////////

//实际就是将局部地图中的点放入到全局地图中去
pcl::PointCloud<pcl::PointXYZI> get_local_map(std::vector<pcl::PointCloud<pcl::PointXYZI>> map_cloud) {

    pcl::PointCloud<pcl::PointXYZI> map_points;

    for (int i = 0; i < map_cloud.size(); i++) {
        map_points += map_cloud[i];
    }


    map_points.header = map_global.header;

    if (map_points.points.size() == 0)
        std::cout << "empty !" << std::endl;
    else
        std::cout << map_cloud.size() << "  " << map_points.points.size() << std::endl;

    return map_points;
}

//订阅原始点云
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {


    ros::Time startT = ros::Time::now();
    //滤波之后的点云
    pcl::PointCloud<pcl::PointXYZRGB> filtered_single_scan;
    //   filtered_single_scan = ps_processor.process_velodyne(input, tfListener);
    //   filtered_single_scan.header.frame_id = "base_link";

    double r;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZRGB> tmp_rgb, scan_selected;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    tf::Quaternion q;
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());


    tf::Transform transform;

    ros::Time scan_time = input->header.stamp;

    tf::StampedTransform velodyne_to_map;

    //tf provides a nice tool that will wait until a transform becomes available
    // 等待直到transform available
    tfListener->waitForTransform("/base_link", input->header.frame_id, ros::Time(0), ros::Duration(0.5));

    //这里建立两个tf之间的关系
    tfListener->lookupTransform("/base_link", input->header.frame_id, ros::Time(0), velodyne_to_map);

    sensor_msgs::PointCloud2 cloud_map;
    Eigen::Matrix4f eigen_transform;

    pcl_ros::transformAsMatrix(velodyne_to_map, eigen_transform);

    std::cout << "eigen_transform" << eigen_transform << std::endl;

    //完成相应的点云变换
    pcl_ros::transformPointCloud(eigen_transform, *input, cloud_map);

    //发布base_link坐标系下的点云
    cloud_map.header.frame_id = "base_link";
    pub_velodyne_base.publish(cloud_map);

    cloud_map.header.frame_id = "map";

    ////////////////////////////////// xi /////////////////////////////////////////
    pcl::fromROSMsg(cloud_map, tmp);
    // pcl::fromROSMsg(cloud_map, tmp_rgb);
    //   pcl::fromROSMsg(*input, tmp);
    ///////////////////////////////////////////////////////////////////////////////

    int cloud_index = 0;


    //将ros点云转换到pcl坐标系下

    //实际相当于滤波，就是保留range> 0的点
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++) {
        p.x = (double) item->z;
        p.y = (double) item->x;
        p.z = (double) item->y;
        p.intensity = (double) item->intensity;

        //这里实际求解的是range, 即x^2+y^2=range
        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));

        //只选择range>2 的点，这样就是将距离太近的点给予过滤
        if (r > RANGE) {
            scan.push_back(p);
        }

        // // select points for other process
        // if (p.x > 0 && abs(p.y) < 5 && p.z < 1)
        // // if(p.x > 0)
        // {
        //   scan_selected.push_back(tmp_rgb.points[cloud_index]);
        // }
        // cloud_index++;
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    // Add initial point cloud to velodyne_map

    //这个if仅仅只执行一次
    if (initial_scan_loaded == 0) {
        //将当前的scan逐帧添加到 map_global中去
        map_global += *scan_ptr;
        initial_scan_loaded = 1;

        //设置局部点云的大小,这里貌似只存储了20帧
        map_local.resize(map_local_length);  // local point cloud buffer for NDT registration
        //每来一帧就对点云进行赋值到map_local[0] 当中
        map_local[0] = *scan_ptr;

        // map_terrain.resize(map_terrain_length);  // local sekected points buffer with color for publish to other
        // processers
        //                                          //   map_terrain[0] = scan_selected;
        // map_terrain[0] = filtered_single_scan;
    }

    // Apply voxelgrid filter

    //滤波之后的点云
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    //  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_global));

    //将局部地图中的点放入到全局地图中去
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(get_local_map(map_local)));

    ROS_INFO_STREAM("Preprocessing takes: " << (ros::Time::now() - startT).toSec() << "s ");

    ros::Time ndt_beginT = ros::Time::now();

//    //为终止条件设置最小转换差异
//    ndt.setTransformationEpsilon(trans_eps);
//
//    //为More-Thuente线搜索设置最大步长
//    ndt.setStepSize(step_size);
//    //设置NDT网格结构的分辨率（VoxelGridCovariance）（体素格的大小）（体素格设置了有1米）
//    ndt.setResolution(ndt_res);
//    //设置匹配迭代的最大次数 目前为30次
//    ndt.setMaximumIterations(iter);
//    //将滤波后的点云，目前是当前帧
//    ndt.setInputSource(filtered_scan_ptr);
//    //实际就是用scan-to-localmap进行匹配
//    ndt.setInputTarget(map_ptr);


//××××××××××××××××××××××××××××××××××××××××采取不同的匹配方法×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××//


    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>());

    ndt_omp->setNumThreads(omp_get_max_threads());
    ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);

    //为终止条件设置最小转换差异
    ndt_omp->setTransformationEpsilon(trans_eps);

    //为More-Thuente线搜索设置最大步长
    ndt_omp->setStepSize(step_size);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）（体素格的大小）（体素格设置了有1米）
    ndt_omp->setResolution(ndt_res);
    //设置匹配迭代的最大次数 目前为30次
    ndt_omp->setMaximumIterations(iter);
    //将滤波后的点云，目前是当前帧
    ndt_omp->setInputSource(filtered_scan_ptr);
    //实际就是用scan-to-localmap进行匹配
    ndt_omp->setInputTarget(map_ptr);

//××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××//











    sensor_msgs::PointCloud2::Ptr local_map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *local_map_msg_ptr);

    local_map_msg_ptr->header.frame_id = "map";
    //这里发布局部地图
    local_map_pub.publish(*local_map_msg_ptr);

    //ndt 两帧点云进行赋初值
    guess_pos.x = previous_pos.x + offset_x;
    guess_pos.y = previous_pos.y + offset_y;
    guess_pos.z = previous_pos.z + offset_z;
    guess_pos.roll = previous_pos.roll;
    guess_pos.pitch = previous_pos.pitch;
    guess_pos.yaw = previous_pos.yaw + offset_yaw;


    //对估计的旋转角度进行归一化
    Eigen::AngleAxisf init_rotation_x(guess_pos.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pos.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pos.yaw, Eigen::Vector3f::UnitZ());

    //设置初始位姿
    Eigen::Translation3f init_translation(guess_pos.x, guess_pos.y, guess_pos.z);

    //设置初始位姿估计 init_guess

    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    // t3_end = ros::Time::now();
    // d3 = t3_end - t3_start;


    t4_start = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    //设置最终的点云匹配结果存储到output中去
   //  ndt.align(*output_cloud, init_guess);

    ndt_omp->align(*output_cloud, init_guess);

    //点云的最终匹配
    //t = ndt.getFinalTransformation();

    t=ndt_omp->getFinalTransformation();

    ROS_INFO_STREAM("NDT takes: " << (ros::Time::now() - ndt_beginT).toSec() << "s ");
    ////////////////////////////////// xi /////////////////////////////////////////
    //******************************小场景稠密建立地图方式*********************************//
    //pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t);

    //******************************大场景稀疏建立地图方式*********************************//
    pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr, t);



    // pcl::transformPointCloud(scan_selected, *transformed_rgb_ptr, t);
    //  pcl::transformPointCloud(filtered_single_scan, *transformed_rgb_ptr, t);
    // cout << transformed_rgb_ptr->points.size() << " " << ps_processor.cloud_reformed_height.points.size();
    //  for(int i = 0; i < filtered_single_scan.size(); i++ )
    //  {
    //      transformed_rgb_ptr->points[i].z = ps_processor.cloud_reformed_height.points[i].z;
    //  }

    ///////////////////////////////////////////////////////////////////////////////
    tf::Matrix3x3 tf3d;

    //这里的t是ndt算法得到的结果
    tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                  static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                  static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

    // Update current_pos.
    current_pos.x = t(0, 3);
    current_pos.y = t(1, 3);
    current_pos.z = t(2, 3);

    tf3d.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw, 1);


    transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
    q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    transform.setRotation(q);

    g_transform = transform;

    //*******************解决TFNormalized*****************************//
    double qx, qy, qz, qw, norm, norm1;
    qx = g_transform.getRotation().x() * g_transform.getRotation().x();
    qy = g_transform.getRotation().y() * g_transform.getRotation().y();
    qz = g_transform.getRotation().z() * g_transform.getRotation().z();
    qw = g_transform.getRotation().w() * g_transform.getRotation().w();
    norm = qx + qy + qz + qw;
    ROS_DEBUG("Pre normalisation, norm=%e", norm);
    long double recipNorm = 1 / sqrt(norm);
    double qx1, qy1, qz1, qw1;
    qx1 = g_transform.getRotation().x();
    qy1 = g_transform.getRotation().y();
    qz1 = g_transform.getRotation().z();
    qw1 = g_transform.getRotation().w();

    qx1 *= recipNorm;
    qy1 *= recipNorm;
    qz1 *= recipNorm;
    qw1 *= recipNorm;

    norm1 = qx1 * qx1 + qy1 * qy1 + qz1 * qz1 + qw1 * qw1;
    ROS_DEBUG("Post normalisation, norm=%e", norm1);

    g_transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
    g_transform.setRotation(tf::Quaternion(qx1, qy1, qz1, qw1));
    //*******************解决TFNormalized*****************************//





    // br.sendTransform(tf::StampedTransform(transform, scan_time, "map", "base_link"));
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "map"));

    // Calculate the offset (curren_pos - previous_pos)
    //计算前后两帧之间的偏差
    offset_x = current_pos.x - previous_pos.x;
    offset_y = current_pos.y - previous_pos.y;
    offset_z = current_pos.z - previous_pos.z;
    offset_yaw = current_pos.yaw - previous_pos.yaw;

    // Update position and posture. current_pos -> previous_pos
    //将当前帧的位姿赋值给上一帧
    previous_pos.x = current_pos.x;
    previous_pos.y = current_pos.y;
    previous_pos.z = current_pos.z;

//  previous_pos.roll = 0.998 *current_pos.roll+0.002*(roll-first_roll);
//  previous_pos.pitch =0.998*current_pos.pitch+0.002*(pitch-first_pitch);
//  previous_pos.yaw = 0.998*current_pos.yaw+0.002*(yaw-first_yaw);


    previous_pos.roll =  current_pos.roll ;
    previous_pos.pitch = current_pos.pitch;
    previous_pos.yaw =  current_pos.yaw;


    std::ofstream f1;

    f1.open("/home/machozhao/catkin_ws/src/ndt_localizer/ndt_omp.txt", ios::app);
    if (!f1) {
        std::cout << "error open file" << std::endl;
    }

    float ndt_roll, ndt_pitch, ndt_yaw, imu_roll, imu_pitch, imu_yaw;
    ndt_roll = current_pos.roll * 180 / 3.1415926;
    ndt_pitch = current_pos.pitch * 180 / 3.1415926;
    ndt_yaw = current_pos.yaw * 180 / 3.1415926;
    imu_roll = (roll - first_roll) * 180 / 3.1415926;
    imu_pitch = (pitch - first_pitch) * 180 / 3.1415926;
    imu_yaw = (yaw - first_yaw) * 180 / 3.1415926;


    f1 << std::setprecision(6) << input->header.stamp << std::setprecision(7) << " " << ndt_roll << " " << ndt_pitch
       << " " << ndt_yaw
       << " " << imu_roll << " " << imu_pitch << " " << imu_yaw << std::endl;
    f1.close();




    //直接使用IMU估计的角度
    // previous_pos.roll =roll;
    // previous_pos.pitch = pitch;
    //  previous_pos.yaw = yaw;



    // Calculate the shift between added_pos and current_pos
    //计算两帧之间的距离
    double shift = sqrt(pow(current_pos.x - added_pos.x, 2.0) + pow(current_pos.y - added_pos.y, 2.0));


    //计算上一帧与当前帧的距离
    //这里的SHIFT实际设置的是1米，也就是当shift>1米时，将变换后的点云添加到 map_global 中，map_global最后又放入到全局点云中

    //实际就是将变换后的点云放入到局部地图中去
    if (shift >= SHIFT) {
        //这里将变换后的每一帧点云，放入到全局地图中去
        map_global += *transformed_scan_ptr;
        added_pos.x = current_pos.x;
        added_pos.y = current_pos.y;
        added_pos.z = current_pos.z;
        added_pos.roll = current_pos.roll;
        added_pos.pitch = current_pos.pitch;
        added_pos.yaw = current_pos.yaw;

        //这里主要是建立局部地图，将转换后的点云加入到局部地图中去
        map_local_index++;
        if (map_local_index == map_local_length)
            map_local_index = 0;
        std::cout << map_local_index << std::endl;
        map_local[map_local_index] = *transformed_scan_ptr;
        /////////////////////////////////////////////////////
    }

    // if (shift > shift_terrain)
    // {
    //   map_terrain_index++;
    //   if (map_terrain_index == map_terrain_length)
    //     map_terrain_index = 0;

    //   map_terrain[map_terrain_index] = *transformed_rgb_ptr;
    //   std::cout << map_terrain_index << " " << map_terrain_length << std::endl;
    // }

    // pcl::PointCloud<pcl::PointXYZRGB> terrain_cloud = get_local_map(map_terrain);

    // map_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(map_global));
    // sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    // pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    // map_msg_ptr->header.frame_id = "map";
    // ndt_map_pub.publish(*map_msg_ptr);

    q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = scan_time;
    current_pose_msg.pose.position.x = current_pos.x;
    current_pose_msg.pose.position.y = current_pos.y;
    current_pose_msg.pose.position.z = current_pos.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();

    current_pose_pub.publish(current_pose_msg);

    ROS_INFO_STREAM("Total takes: " << (ros::Time::now() - startT).toSec() << "s ");

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
    std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
    std::cout << "map: " << map_global.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << ndt_omp->hasConverged() << std::endl;
    std::cout << "Fitness score: " << ndt_omp->getFitnessScore() << std::endl;
    std::cout << "Number of iteration: " << ndt_omp->getFinalNumIteration() << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ", " << current_pos.roll
              << ", " << current_pos.pitch << ", " << current_pos.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
}

void handleIMUMessage(const sensor_msgs::Imu::ConstPtr &imuIn) {
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    if (is_FirstImu) {

        //×××××××××××××××××××××××××××××记录IMU的第一帧数据×××××××××××××××××××××××××××××××××××××××××××//
        tf::Matrix3x3(orientation).getRPY(first_roll, first_pitch, first_yaw);
        is_FirstImu = false;
        return;
    } else {

        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    }

}

static void publish_map(const ros::TimerEvent &event) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>(map_global));
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    ndt_map_pub.publish(*map_msg_ptr);
}

int main(int argc, char **argv) {

    //完成实现初始化步骤
    previous_pos.x = 0.0;
    previous_pos.y = 0.0;
    previous_pos.z = 0.0;
    previous_pos.roll = 0.0;
    previous_pos.pitch = 0.0;
    previous_pos.yaw = 0.0;

    current_pos.x = 0.0;
    current_pos.y = 0.0;
    current_pos.z = 0.0;
    current_pos.roll = 0.0;
    current_pos.pitch = 0.0;
    current_pos.yaw = 0.0;

    guess_pos.x = 0.0;
    guess_pos.y = 0.0;
    guess_pos.z = 0.0;
    guess_pos.roll = 0.0;
    guess_pos.pitch = 0.0;
    guess_pos.yaw = 0.0;

    //增加位姿步骤
    added_pos.x = 0.0;
    added_pos.y = 0.0;
    added_pos.z = 0.0;
    added_pos.roll = 0.0;
    added_pos.pitch = 0.0;
    added_pos.yaw = 0.0;

    offset_x = 0.0;
    offset_y = 0.0;
    offset_z = 0.0;
    offset_yaw = 0.0;

    ros::init(argc, argv, "ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(9);


    //设置量程和位移
    private_nh.getParam("range", RANGE);
    std::cout << "RANGE: " << RANGE << std::endl;
    private_nh.getParam("shift", SHIFT);
    std::cout << "SHIFT: " << SHIFT << std::endl;
    ///////////////////////////////////// Xi ////////////////////////

    // setting parameters
    //设置量程和位移
    private_nh.getParam("map_local_length", map_local_length);
    std::cout << "map_local_length: " << map_local_length << std::endl;
    private_nh.getParam("map_terrain_length", map_terrain_length);
    std::cout << "map_terrain_length: " << map_terrain_length << std::endl;
    private_nh.getParam("shift_terrain", shift_terrain);
    std::cout << "shift_terrain: " << shift_terrain << std::endl;
    //////////////////////////////////////////////////////////////////////

    map_global.header.frame_id = "map";
    tfListener = new (tf::TransformListener);

    //这里发布的全局地图
    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1000);

    timer = nh.createTimer(ros::Duration(1), publish_map);

    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
    pub_velodyne_base = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_base", 1);

    //   ros::Subscriber param_sub = nh.subscribe("config/ndt_mapping", 10, param_callback);
    //   ros::Subscriber output_sub = nh.subscribe("config/ndt_mapping_output", 10, output_callback);
    ros::Subscriber points_sub = nh.subscribe("points_raw", 1, points_callback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, handleIMUMessage);
    tf::TransformBroadcaster br;

    while (ros::ok()) {
        br.sendTransform(tf::StampedTransform(g_transform.inverse(), ros::Time::now(), "base_link", "map"));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
