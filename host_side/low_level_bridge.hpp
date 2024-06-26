#ifndef LOW_LEVEL_BRIDGE_HPP
#define LOW_LEVEL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <array>

// Serial communication configs
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE 115200
#define HEADER 0xC8
#define TAIL 0xC7
#define SIZE_OF_RX_DATA 52
#define SIZE_OF_TX_DATA 52
#define QUEUE_CAPACITY 100

// ROS configs
#define PUBLISHED_ODOMETRY_TOPIC_NAME "odometry/wheel_encoders"
#define SUBSCRIBED_TWIST_TOPIC_NAME "cmd_vel"
#define SUBSCRIBED_ODOMETRY_TOPIC_NAME "odometry/filtered"
#define PARENT_FRAME_ID "base_link"

typedef enum {
    CMD_VEL = 1,
    ODOMETRY = 2
} MessageType_e;

typedef struct {
    MessageType_e type;
    std::array<uint8_t, SIZE_OF_TX_DATA> data;
} SerialMessage_t;

class OdometryProcessor : public rclcpp::Node {
public:
    OdometryProcessor();
    ~OdometryProcessor();
    void spin();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void serialThread();
    void sendSerialData(const SerialMessage_t& msg);
    void processReceivedData(const std::vector<uint8_t>& buffer);
    bool enqueueMessage(const SerialMessage_t& msg);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    boost::thread serial_thread_;
    std::string serial_port_;
    int baud_rate_;

    boost::lockfree::queue<SerialMessage_t> message_queue_{QUEUE_CAPACITY};
};

#endif // LOW_LEVEL_BRIDGE_HPP
