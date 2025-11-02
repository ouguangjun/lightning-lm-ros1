//
// Created by xiang on 23-12-14.
//

#ifndef LIGHTNING_BAG_IO_H
#define LIGHTNING_BAG_IO_H

#include <functional>
#include <map>
#include <string>
#include <csignal>

#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include "livox_ros_driver/CustomMsg.h"

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "io/dataset_type.h"
#include "wrapper/ros_utils.h"

namespace lightning {

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 * 现在可以指定ROS2
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)) {
        /// handle ctrl-c
        signal(SIGINT, lightning::debug::SigHandle);
    }

    using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;

    /// 一些方便直接使用的topics, messages
    using Scan2DHandle = std::function<bool(const sensor_msgs::LaserScan::ConstPtr &)>;

    using PointCloud2Handle = std::function<bool(const sensor_msgs::PointCloud2::ConstPtr &)>;
    using LivoxCloud2Handle = std::function<bool(const livox_ros_driver::CustomMsg::ConstPtr &)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using OdomHandle = std::function<bool(const OdomPtr &)>;

    /**
     * 遍历文件内容，调用回调函数
     * @param sleep_usec 每调用一个回调后的等待时间
     */
    void Go(int sleep_usec = 0);

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// point cloud 2 处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (!msg) {
                return false;
            }
            return f(msg);
        });
    }

    /// livox 处理
    RosbagIO &AddLivoxCloudHandle(const std::string &topic_name, LivoxCloud2Handle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (!msg) {
                return false;
            }
            return f(msg);
        });
    }

    RosbagIO &AddImuHandle(const std::string &topic_name, ImuHandle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<sensor_msgs::Imu>();
            if (!msg) {
                return false;
            }
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            return f(imu);
        });
    }

    /// odom 处理
    // RosbagIO &AddOdomHandle(const std::string &topic_name, OdomHandle f) {
    //     return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
    //         auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    //         rclcpp::SerializedMessage data(*m->serialized_data);
    //         seri_odom_.deserialize_message(&data, msg.get());

    //         /// nav_msg 的 odometry 转 odom
    //         return f(msg);
    //     });
    // }

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

   private:
    std::map<std::string, MessageProcessFunction> process_func_;

    std::string bag_file_;
    DatasetType dataset_type_ = DatasetType::NCLT;
};
}  // namespace lightning

#endif  // SLAM_ROS_BAG_IO_H
