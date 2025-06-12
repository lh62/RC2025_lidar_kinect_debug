#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/UInt8.h>
#include <serial/serial.h>
#include <tf/tf.h>
#include <cmath>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h>

struct __attribute__((packed)) DataPacket {
    uint8_t header[2] = {0x0A, 0x0D};
    float kinect_loop_depth = 10000;
    float kinect_loop_dx = 10000;
    float kinect_circle_x = 0;
    float kinect_circle_y = 0;
    float base_link_x = 0;
    float base_link_y = 0;
    float base_link_w = 0;
    uint8_t tail[2] = {0x0B, 0x0C};
};

struct __attribute__((packed)) check_point {
    float x;
    float y;
};

class SerialSender {
public:
    SerialSender(ros::NodeHandle& nh) : ser() {
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        nh.param<size_t>("baud_rate", baud_rate, 115200);
        nh.param<std::string>("base_link_pub", base_link_pub, "/robot_pose_in_world");
        nh.param<std::string>("kinect_loop_pub", kinect_loop_pub, "/kinect/loop/targets_in_world");
        nh.param<std::string>("kinect_circle_pub", kinect_circle_pub, "/kinect/circle/targets_in_world");
        nh.param<bool>("en_kinect_loop", en_kinect_loop, true);
        nh.param<bool>("en_kinect_circle", en_kinect_circle, true);
        nh.param<bool>("en_base_link", en_base_link, true);
        nh.param<float>("arrive_circle_point_distance_threshold", arrive_circle_point_distance_threshold, 0.5);
        nh.param<float>("arrive_circle_point_x_threshold", arrive_circle_point_x_threshold, 0.1);
        nh.param<float>("arrive_circle_point_y_threshold", arrive_circle_point_y_threshold, 0.1);


        if (en_base_link)
            sub_base_link = nh.subscribe(base_link_pub, 1, &SerialSender::sub_base_link_Callback, this);
        if (en_kinect_loop)
            sub_kinect_loop = nh.subscribe(kinect_loop_pub, 1, &SerialSender::sub_kinect_loop_Callback, this);
        if (en_kinect_circle)
            sub_kinect_circle = nh.subscribe(kinect_circle_pub, 1, &SerialSender::sub_kinect_circle_Callback, this);

        data_packet.kinect_loop_depth = 10000.0;
        data_packet.kinect_loop_dx = 10000.0;
        data_packet.kinect_circle_x = 0.0;
        data_packet.kinect_circle_y = 0.0;
        data_packet.base_link_x = 0.0;
        data_packet.base_link_y = 0.0;
        data_packet.base_link_w = 0.0;

        try {
            ser.setPort(port);
            ser.setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("无法打开串口: " << e.what());
        }

        if (ser.isOpen()) {
            ROS_INFO("串口成功打开");
        } else {
            ROS_ERROR("串口打开失败");
        }
    }

    void sub_base_link_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        data_packet.base_link_x = msg->pose.position.x * 1000;
        data_packet.base_link_y = msg->pose.position.y * 1000;
        double t_roll, t_pitch, t_yaw;
        tf::Quaternion tf_q;
        tf::quaternionMsgToTF(msg->pose.orientation, tf_q);
        tf::Matrix3x3(tf_q).getRPY(t_roll, t_pitch, t_yaw);
        data_packet.base_link_w = t_yaw * 180 / M_PI;
    }

    void sub_kinect_loop_Callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg) {
        if (msg->detections.size() != 1 || msg->detections.empty()) {
            data_packet.kinect_loop_depth = 50000;
            data_packet.kinect_loop_dx = 50000;
            ROS_WARN("loop_detection_num != 1");
        } else {
            data_packet.kinect_loop_depth = msg->detections[0].point_3d.z * 1000;
            data_packet.kinect_loop_dx = msg->detections[0].dx;
        }
    }

    void sub_kinect_circle_Callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg) {
        if (msg->detections.empty()) {
            data_packet.kinect_circle_x = 0;
            data_packet.kinect_circle_y = 0;
            return;
        }

        // 更新打卡点列表
        for (const auto& detection : msg->detections) {
            double dx = detection.point_3d.x - data_packet.base_link_x / 1000.0;
            double dy = detection.point_3d.y - data_packet.base_link_y / 1000.0;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < arrive_circle_point_distance_threshold * arrive_circle_point_distance_threshold &&
                !is_in_arrived_list(detection)) {
                arrive_circle_points.push_back(check_point{detection.point_3d.x, detection.point_3d.y});
            }
        }

        //找出非打卡点中距离底盘最近的点
        double min_dist_sq = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto& detection : msg->detections) {
            if (!is_in_arrived_list(detection)) {
                double dx = detection.point_3d.x - data_packet.base_link_x / 1000.0;
                double dy = detection.point_3d.y - data_packet.base_link_y / 1000.0;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    data_packet.kinect_circle_x = detection.point_3d.x * 1000;
                    data_packet.kinect_circle_y = detection.point_3d.y * 1000;
                    found = true;
                }
            }
        }

        if (!found) {
            data_packet.kinect_circle_x = 0;
            data_packet.kinect_circle_y = 0;
        }
    }

    void sendData() {
        if (ser.isOpen()) {
            ser.write(reinterpret_cast<uint8_t*>(&data_packet), sizeof(data_packet));
        } else {
            ROS_WARN("串口未打开");
        }
    }

    void print_info() {
        ROS_INFO("kinect_loop_depth: %.3f", this->data_packet.kinect_loop_depth);
        ROS_INFO("kinect_loop_dx: %.3f", this->data_packet.kinect_loop_dx);
        ROS_INFO("kinect_circle_x: %.3f", this->data_packet.kinect_circle_x);
        ROS_INFO("kinect_circle_y: %.3f", this->data_packet.kinect_circle_y);
        ROS_INFO("base_link_x: %.3f", this->data_packet.base_link_x);
        ROS_INFO("base_link_y: %.3f", this->data_packet.base_link_y);
        ROS_INFO("base_link_w: %.3f", this->data_packet.base_link_w);
    }

    bool is_in_arrived_list(const yolo_realsense_kinect::DetectedObject3D_kinect_circle& detection) {
        for (const auto& point : arrive_circle_points) {
            if (std::abs(point.x - detection.point_3d.x) < arrive_circle_point_x_threshold &&
                std::abs(point.y - detection.point_3d.y) < arrive_circle_point_y_threshold) {
                return true;
            }
        }
        return false;
    }

private:
    ros::Subscriber sub_base_link;
    ros::Subscriber sub_kinect_loop;
    ros::Subscriber sub_kinect_circle;
    std::string base_link_pub;
    std::string kinect_loop_pub;
    std::string kinect_circle_pub;
    bool en_kinect_loop;
    bool en_kinect_circle;
    bool en_base_link;
    serial::Serial ser;
    DataPacket data_packet;
    size_t baud_rate;
    std::string port;
    std::vector<check_point> arrive_circle_points;
    float arrive_circle_point_distance_threshold;
    float arrive_circle_point_x_threshold;
    float arrive_circle_point_y_threshold;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_kinect_lidar");
    ros::NodeHandle nh;
    SerialSender sender(nh);

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();  // 先处理回调，更新data_packet
        sender.sendData(); // 再发送更新后的数据
        sender.print_info();
        rate.sleep();
    }

    return 0;
}