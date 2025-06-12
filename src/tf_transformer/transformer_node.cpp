#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于 tf2::doTransform
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h> // 包含自定义消息的头文件
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h> // 包含自定义消息的头文件
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h> // 包含自定义消息的头文件
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h> // 包含自定义消息的头文件
#include <target_world_transformer/World_Kinect_Target_Messages.h> //包含自定义消息的头文件
#include <target_world_transformer/World_Kinect_Target_MessagesArray.h> //包含自定义消息的头文件
#include <target_world_transformer/World_Realsense_Target_Messages.h> //包含自定义消息的头文件
#include <target_world_transformer/World_Realsense_Target_MessagesArray.h> //包含自定义消息的头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <cmath> // For M_PI
#include <map>

// 用于将 roll, pitch, yaw 转换为四元数
tf2::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

class TargetWorldTransformer {
public:
    TargetWorldTransformer(ros::NodeHandle& nh) :
        nh_(nh),
        tf_listener_(tf_buffer_)
    {
        // --- 从参数服务器加载帧 ID ---
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
        nh_.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
        nh_.param<std::string>("kinect_camera_optical_frame_id", kinect_camera_optical_frame_id_, "kinect_color_optical_frame");
        nh_.param<std::string>("realsense_camera_optical_frame_id", realsense_camera_optical_frame_id_, "realsense_color_optical_frame");
        nh_.param<std::string>("lidar_sensor_frame_id", lidar_sensor_frame_id_, "lidar_link"); // 安装在 base_link 上的雷达传感器的参考帧
        nh_.param<std::string>("lidar_map_origin_frame_id", lidar_map_origin_frame_id_, "camera_init"); // 雷达SLAM的固定地图原点
        nh_.param<std::string>("lidar_current_pose_frame_id", lidar_current_pose_frame_id_, "aft_mapped"); // 雷达在 camera_init 中的当前位姿帧

        // --- 从参数服务器加载静态变换参数 (x, y, z, roll, pitch, yaw) ---
        // 1. world -> camera_init (雷达地图原点在世界坐标系下的位姿)
        load_static_transform_param("world_to_lidar_map_origin", world_frame_id_, lidar_map_origin_frame_id_);

        // 2. base_link -> kinect_camera_optical_frame (Kinect相机在底盘上的安装位置)
        load_static_transform_param("base_link_to_kinect_camera", base_link_frame_id_, kinect_camera_optical_frame_id_);

        // 3. base_link -> realsense_camera_optical_frame (RealSense相机在底盘上的安装位置)
        load_static_transform_param("base_link_to_realsense_camera", base_link_frame_id_, realsense_camera_optical_frame_id_);

        // 4. base_link -> lidar_sensor_frame (LiDAR传感器本身在其安装板/参考帧上的位置，这个安装板/参考帧的位姿由aft_mapped给出)
        // 这里的参数 "base_link_to_lidar_sensor" 定义了 `lidar_sensor_frame_id_` 相对于 `base_link_frame_id_` 的变换。
        // `aft_mapped` 给出了 `lidar_sensor_frame_id_` 在 `lidar_map_origin_frame_id_` (`camera_init`) 中的位姿。
        load_static_transform_param("base_link_to_lidar_sensor", base_link_frame_id_, lidar_sensor_frame_id_);

        // 发布这些静态变换
        if (!static_transforms_.empty()) {
            static_broadcaster_.sendTransform(static_transforms_); // 使用StaticTransformBroadcaster发布一次即可
            ROS_INFO("已成功发布 %zu 个静态TF变换。", static_transforms_.size());
        } else {
            ROS_WARN("没有从参数服务器加载到任何有效的静态TF变换。TF树可能不完整。");
        }

        // --- ROS 订阅器 ---
        sub_kinect_detections_ = nh_.subscribe("/yolo/kinect/detections_3d", 10, &TargetWorldTransformer::kinect_detections_callback, this);
        sub_realsense_detections_ = nh_.subscribe("/yolo/realsense/detections_3d", 10, &TargetWorldTransformer::realsense_detections_callback, this);

        // --- ROS 发布器 ---
        pub_base_link_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose_in_world", 10);
        pub_kinect_targets_world_ = nh_.advertise<target_world_transformer::World_Kinect_Target_MessagesArray>("/kinect/targets_in_world", 10);
        pub_realsense_targets_world_ = nh_.advertise<target_world_transformer::World_Realsense_Target_MessagesArray>("/realsense/targets_in_world", 10);
        pub_kinect_targets_world_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/targets_in_world_marker_debug", 10);
        pub_realsense_targets_world_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/realsense/targets_in_world_marker_debug", 10);
        // --- 定时器用于发布机器人位姿 ---
        // TF 的 `camera_init` -> `aft_mapped` 是动态的，我们会用 lookupTransform 获取
        // 然后通过已知的静态变换链计算 base_link 在 world 中的位姿
        pose_publish_timer_ = nh_.createTimer(ros::Duration(1.0/30.0), &TargetWorldTransformer::publish_robot_pose_callback, this); // 30 Hz

        // 初始化目标点历史记录
        target_history_.clear();
        last_update_time_ = ros::Time::now();

        // 从参数服务器加载过滤参数
        nh_.param<double>("target_distance_threshold", target_distance_threshold_, 0.3); // 目标点距离阈值（米）
        nh_.param<double>("target_timeout", target_timeout_, 5.0); // 目标点超时时间（秒）
        nh_.param<double>("target_confidence_threshold", target_confidence_threshold_, 0.6); // 目标点置信度阈值
        nh_.param<double>("target_position_threshold", target_position_threshold_, 0.1); // 目标点位置变化阈值（米）
        nh_.param<int>("target_history_size", target_history_size_, 10); // 目标点历史记录大小

        ROS_INFO("目标点过滤参数：距离阈值=%.2fm, 超时时间=%.1fs, 置信度阈值=%.2f, 位置变化阈值=%.2fm",
                 target_distance_threshold_, target_timeout_, target_confidence_threshold_, target_position_threshold_);

        ROS_INFO("目标点世界坐标转换节点已初始化。");
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> static_transforms_; // 存储从参数加载的静态TF

    ros::Subscriber sub_kinect_detections_;
    ros::Subscriber sub_realsense_detections_;
    ros::Publisher pub_realsense_debug_markers_;       // 发布 Rviz 可视化标记
    ros::Publisher pub_kinect_debug_markers_;       // 发布 Rviz 可视化标记
    ros::Publisher pub_base_link_pose_;
    ros::Publisher pub_kinect_targets_world_;
    ros::Publisher pub_realsense_targets_world_;
    ros::Timer pose_publish_timer_;

    // 帧 ID
    std::string world_frame_id_;
    std::string base_link_frame_id_;
    std::string kinect_camera_optical_frame_id_;
    std::string realsense_camera_optical_frame_id_;
    std::string lidar_sensor_frame_id_; // 雷达传感器在 base_link 上的坐标系名称
    std::string lidar_map_origin_frame_id_; // e.g., "camera_init"
    std::string lidar_current_pose_frame_id_; // e.g., "aft_mapped"

    // 目标点历史记录结构
    struct TargetHistory {
        geometry_msgs::Point position;
        ros::Time last_seen;
        bool is_processed;
        std::vector<geometry_msgs::Point> position_history;
    };
    std::map<int, TargetHistory> target_history_; // 目标点ID到历史记录的映射
    ros::Time last_update_time_;
    
    // 过滤参数
    double target_distance_threshold_;    // 目标点距离阈值
    double target_timeout_;              // 目标点超时时间
    double target_confidence_threshold_;  // 目标点置信度阈值
    double target_position_threshold_;    // 目标点位置变化阈值
    int target_history_size_;            // 目标点历史记录大小

    // 从 launch 文件加载静态变换参数并存储
    void load_static_transform_param(const std::string& param_name_prefix, const std::string& parent_frame, const std::string& child_frame) {
        std::vector<double> transform_values; // x, y, z, roll_deg, pitch_deg, yaw_deg
        if (nh_.getParam(param_name_prefix, transform_values) && transform_values.size() == 6) {
            geometry_msgs::TransformStamped T_stamped;
            T_stamped.header.stamp = ros::Time::now();
            T_stamped.header.frame_id = parent_frame;
            T_stamped.child_frame_id = child_frame;
            T_stamped.transform.translation.x = transform_values[0];
            T_stamped.transform.translation.y = transform_values[1];
            T_stamped.transform.translation.z = transform_values[2];

            tf2::Quaternion q = rpy_to_quaternion(
                transform_values[3] * M_PI / 180.0, // roll in radians
                transform_values[4] * M_PI / 180.0, // pitch in radians
                transform_values[5] * M_PI / 180.0  // yaw in radians
            );
            T_stamped.transform.rotation.x = q.x();
            T_stamped.transform.rotation.y = q.y();
            T_stamped.transform.rotation.z = q.z();
            T_stamped.transform.rotation.w = q.w();
            static_transforms_.push_back(T_stamped);
            ROS_INFO("已加载静态变换 %s -> %s", parent_frame.c_str(), child_frame.c_str());
        } else {
            ROS_ERROR("未能从参数服务器加载静态变换 '%s'，或参数格式不正确 (需要6个值: x,y,z,roll_deg,pitch_deg,yaw_deg)。将使用默认单位变换（如果适用）。", param_name_prefix.c_str());
            // 可以选择添加一个单位变换作为默认值，以避免程序崩溃，但这取决于具体需求
             geometry_msgs::TransformStamped T_stamped;
            T_stamped.header.stamp = ros::Time::now();
            T_stamped.header.frame_id = parent_frame;
            T_stamped.child_frame_id = child_frame;
            T_stamped.transform.translation.x = 0;
            T_stamped.transform.translation.y = 0;
            T_stamped.transform.translation.z = 0;
            T_stamped.transform.rotation.x = 0;
            T_stamped.transform.rotation.y = 0;
            T_stamped.transform.rotation.z = 0;
            T_stamped.transform.rotation.w = 1;
            static_transforms_.push_back(T_stamped); // 添加单位变换作为备用
        }
    }

    // 定时回调，发布机器人底盘在世界坐标系下的位姿
    void publish_robot_pose_callback(const ros::TimerEvent&) {
        geometry_msgs::TransformStamped T_world_base;
        try {
            // TF 树应该包含: world -> camera_init (static) -> aft_mapped (dynamic from lidar) -> lidar_sensor_frame (static inverse) -> base_link (static)
            // lookupTransform 的目标是 base_link，源是 world
            // 这要求 lidar_sensor_frame_id_ 的父级是 aft_mapped (这由雷达节点完成 camera_init -> aft_mapped)
            // 并且 base_link 的父级是 lidar_sensor_frame_id_ (这由我们的静态TF base_link_to_lidar_sensor 的逆完成，
            // 或者更直接地，我们发布的 lidar_sensor_frame_id_ -> base_link 的静态TF)
            // 实际上，我们发布 base_link -> lidar_sensor_frame。TF树会构建 lidar_sensor_frame <- base_link
            // 雷达发布 camera_init -> aft_mapped (aft_mapped 就是 lidar_sensor_frame 在 camera_init 中的位姿)
            // 我们发布 world -> camera_init
            // 所以 TF 链是: world -> camera_init -> aft_mapped (即 lidar_sensor_frame) <- base_link
            // 因此，我们可以查找 world 到 base_link
            T_world_base = tf_buffer_.lookupTransform(world_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(0.2));
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "获取 TF 变换 %s -> %s 失败: %s", world_frame_id_.c_str(), base_link_frame_id_.c_str(), ex.what());
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now(); // T_world_base.header.stamp; // 使用查找到的变换的时间戳更准确
        pose_stamped.header.frame_id = world_frame_id_;
        pose_stamped.pose.position.x = T_world_base.transform.translation.x;
        pose_stamped.pose.position.y = T_world_base.transform.translation.y;
        pose_stamped.pose.position.z = T_world_base.transform.translation.z;
        pose_stamped.pose.orientation = T_world_base.transform.rotation;

        pub_base_link_pose_.publish(pose_stamped);

        // (可选) 也可以在这里打印欧拉角
        tf2::Quaternion q(
            T_world_base.transform.rotation.x,
            T_world_base.transform.rotation.y,
            T_world_base.transform.rotation.z,
            T_world_base.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_DEBUG_THROTTLE(1.0, "机器人位姿 (RPY deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }

    // Kinect 目标检测回调
    void kinect_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array; // 创建 Rviz 标记数组消息
        process_detections_kinect(msg, kinect_camera_optical_frame_id_, pub_kinect_targets_world_, marker_array);
        if( marker_array.markers.size()!=0){
            pub_kinect_debug_markers_.publish(marker_array);
        }
        
    }

    // RealSense 目标检测回调
    void realsense_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array; // 创建 Rviz 标记数组消息
        process_detections_realsense(msg, realsense_camera_optical_frame_id_, pub_realsense_targets_world_, marker_array);
        if(marker_array.markers.size()!=0){
        pub_realsense_debug_markers_.publish(marker_array);}
    }

    // 检查目标点是否稳定
    bool is_target_stable(const geometry_msgs::Point& current_pos, const std::vector<geometry_msgs::Point>& history) {
        if (history.empty()) return true;
        
        // 计算与历史位置的平均距离
        double total_distance = 0.0;
        for (const auto& pos : history) {
            total_distance += std::sqrt(
                std::pow(current_pos.x - pos.x, 2) +
                std::pow(current_pos.y - pos.y, 2) +
                std::pow(current_pos.z - pos.z, 2)
            );
        }
        double avg_distance = total_distance / history.size();
        
        return avg_distance < target_position_threshold_;
    }

    // 更新目标点历史记录
    void update_target_history(int target_id, const geometry_msgs::Point& position, bool is_processed = false) {
        auto& history = target_history_[target_id];
        history.position = position;
        history.last_seen = ros::Time::now();
        history.is_processed = is_processed;
        
        // 更新位置历史
        history.position_history.push_back(position);
        if (history.position_history.size() > target_history_size_) {
            history.position_history.erase(history.position_history.begin());
        }
    }

    // 清理过期的目标点历史记录
    void cleanup_target_history() {
        ros::Time current_time = ros::Time::now();
        for (auto it = target_history_.begin(); it != target_history_.end();) {
            if ((current_time - it->second.last_seen).toSec() > target_timeout_) {
                it = target_history_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 检查目标点是否已处理
    bool is_target_processed(int target_id, const geometry_msgs::Point& position) {
        auto it = target_history_.find(target_id);
        if (it == target_history_.end()) return false;
        
        // 检查距离是否小于阈值
        double distance = std::sqrt(
            std::pow(position.x - it->second.position.x, 2) +
            std::pow(position.y - it->second.position.y, 2) +
            std::pow(position.z - it->second.position.z, 2)
        );
        
        return it->second.is_processed && distance < target_distance_threshold_;
    }

    // 修改目标点发布判断函数
    bool should_publish_target_point_common(const geometry_msgs::PointStamped& point_in_world,
                                         const std::string& class_name,
                                         float confidence,
                                         visualization_msgs::MarkerArray& marker_array,
                                         int& marker_id_counter,
                                         const std::string& ns_prefix) {
        // 基本过滤条件
        if (confidence < target_confidence_threshold_) {
            ROS_DEBUG("目标点置信度 %.2f 低于阈值 %.2f，跳过", confidence, target_confidence_threshold_);
            return false;
        }

        // 生成目标点ID（使用位置信息）
        int target_id = static_cast<int>(point_in_world.point.x * 1000 + point_in_world.point.y * 1000);

        // 检查是否已处理
        if (is_target_processed(target_id, point_in_world.point)) {
            ROS_DEBUG("目标点 (ID: %d) 已处理，跳过", target_id);
            return false;
        }

        // 检查目标点稳定性
        auto it = target_history_.find(target_id);
        if (it != target_history_.end() && !is_target_stable(point_in_world.point, it->second.position_history)) {
            ROS_DEBUG("目标点 (ID: %d) 位置不稳定，跳过", target_id);
            return false;
        }

        // 更新目标点历史记录
        update_target_history(target_id, point_in_world.point);

        // 创建可视化标记
        visualization_msgs::Marker marker;
        marker.header = point_in_world.header;
        marker.ns = ns_prefix + "_world_markers";
        marker.id = marker_id_counter++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point_in_world.point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15; marker.scale.y = 0.15; marker.scale.z = 0.15;
        marker.color.a = 0.8;
        if (ns_prefix == "kinect_circle_detections") {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
        }
        marker.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(marker);

        return true;
    }

    // 修改处理函数，添加历史记录清理
    void process_detections_realsense(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg_in,
                                    const std::string& camera_optical_frame,
                                    ros::Publisher& publisher,
                                    visualization_msgs::MarkerArray& marker_array) {
        // 清理过期的目标点历史记录
        cleanup_target_history();

        if (msg_in->detections.empty()) {
            return;
        }

        target_world_transformer::World_Realsense_Target_MessagesArray msg_out;
        msg_out.header.stamp = ros::Time::now(); // 使用当前时间作为输出消息的时间戳
        msg_out.header.frame_id = world_frame_id_; // 输出坐标系为世界坐标系

        for (const auto& det_in : msg_in->detections) {
            // 检查输入点坐标是否有效 (例如，不是 NaN)
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z)) {
                ROS_DEBUG("检测到无效的输入目标点坐标 (NaN)，跳过。");
                continue;
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame; // 输入点在相机光心坐标系
            pt_cam.header.stamp = msg_in->header.stamp; // 使用输入消息的时间戳进行变换
            pt_cam.point = det_in.point_3d;

            try {
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.2)); // 容忍0.2秒的 TF 延迟
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "目标点 TF 变换 %s -> %s 失败: %s", camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue; // 跳过这个点
            }

            // --- 条件判断函数 ---
            if (should_publish_target_point_common(pt_world, det_in.class_name, det_in.confidence, marker_array, marker_id_counter_, "realsense_circle_detections")) {
                target_world_transformer::World_Realsense_Target_Messages det_out ;
                det_out.header = msg_out.header;
                det_out.y = pt_world.point.y; // 更新为世界坐标
                det_out.x = pt_world.point.x;
                msg_out.detections.push_back(det_out);
            }
        }

        if (!msg_out.detections.empty()) {
            publisher.publish(msg_out);
        }
    }

    void process_detections_kinect(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg_in,
                                 const std::string& camera_optical_frame,
                                 ros::Publisher& publisher,
                                 visualization_msgs::MarkerArray& marker_array) {
        // 清理过期的目标点历史记录
        cleanup_target_history();

        if (msg_in->detections.empty()) {
            return;
        }

        target_world_transformer::World_Kinect_Target_MessagesArray msg_out;
        msg_out.header.stamp = ros::Time::now(); // 使用当前时间作为输出消息的时间戳
        msg_out.header.frame_id = world_frame_id_; // 输出坐标系为世界坐标系

        for (const auto& det_in : msg_in->detections) {
            // 检查输入点坐标是否有效 (例如，不是 NaN)
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z)) {
                ROS_DEBUG("检测到无效的输入目标点坐标 (NaN)，跳过。");
                continue;
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame; // 输入点在相机光心坐标系
            pt_cam.header.stamp = msg_in->header.stamp; // 使用输入消息的时间戳进行变换
            pt_cam.point = det_in.point_3d;

            try {
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.2)); // 容忍0.2秒的 TF 延迟
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "目标点 TF 变换 %s -> %s 失败: %s", camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue; // 跳过这个点
            }

            // --- 条件判断函数 ---
            if (should_publish_target_point_common(pt_world, det_in.class_name, det_in.confidence, marker_array, marker_id_counter_, "kinect_loop_detections")) {
                target_world_transformer::World_Kinect_Target_Messages det_out ;
                det_out.header = msg_out.header;
                det_out.dx = det_in.dx; // 更新为世界坐标
                det_out.hoop_net_depth = det_in.point.x;
                msg_out.detections.push_back(det_out);
            }
        }

        if (!msg_out.detections.empty()) {
            publisher.publish(msg_out);
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "target_world_transformer_node");
    ros::NodeHandle nh("~"); // 使用私有节点句柄以获取参数
    TargetWorldTransformer transformer(nh);
    ros::spin();
    return 0;
}