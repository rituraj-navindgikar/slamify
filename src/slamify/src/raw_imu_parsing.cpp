/* parse raw imu data into imu_msgs format
std_msgs/Header header
sensor_msgs/Imu imu
sensor_msgs/MagneticField mag_field
string raw_imu
*/
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"


using std::placeholders::_1;

class ImuSubscriber : public rclcpp::Node {
    public:
    ImuSubscriber() : Node("imu_node") {
        startSubscription("imu_raw", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>
        ("/imu_raw", 10, std::bind(&ImuSubscriber::imu_callback, this, _1));
    }

    private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Angular Velocity - x: " << msg.angular_velocity.x << " " << "y: " << msg.angular_velocity.y << " " << "z: " << msg.angular_velocity.z);
        RCLCPP_INFO(this->get_logger(), "Linear Acceleration - x: %.2f", msg->linear_acceleration.x);
    }

    void startSubscription(const std::string& topic_name, int queue_size) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, queue_size, std::bind(&ImuSubscriber::imu_callback, this, std::placeholders::_1));
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubscriber>());
    rclcpp::shutdown();
    return 0;
}








