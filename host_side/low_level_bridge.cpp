#include "base/low_level_bridge.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void packFloatIntoArray(std::array<uint8_t, SIZE_OF_TX_DATA>& data, float value, size_t offset) {
    if (offset + sizeof(float) <= data.size()) {
        std::memcpy(&data[offset], &value, sizeof(float));
    }
}

OdometryProcessor::OdometryProcessor()
    : Node("odometry_processor"), serial_(io_), serial_port_(SERIAL_PORT), baud_rate_(BAUD_RATE) {

    // Setup ROS publishers and subscribers
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(PUBLISHED_ODOMETRY_TOPIC_NAME, 10);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(SUBSCRIBED_TWIST_TOPIC_NAME, 10, std::bind(&OdometryProcessor::twistCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(SUBSCRIBED_ODOMETRY_TOPIC_NAME, 10, std::bind(&OdometryProcessor::odometryCallback, this, std::placeholders::_1));

    // Setup serial port
    serial_.open(serial_port_);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

    // Start the serial communication thread
    serial_thread_ = boost::thread(boost::bind(&OdometryProcessor::serialThread, this));
}

OdometryProcessor::~OdometryProcessor() {
    io_.stop();
    serial_thread_.join();
    serial_.close();
}

bool OdometryProcessor::enqueueMessage(const SerialMessage_t& msg) {
    if (!message_queue_.push(msg)) {
        // Queue is full, pop one item to make space
       
        SerialMessage_t discarded_msg;

        if (!message_queue_.pop(discarded_msg)) {
            RCLCPP_WARN(this->get_logger(), "Failed to pop from the queue even though it's reported as full.");
            return false;
        }

        // Try again to push the new message
        if (!message_queue_.push(msg)) {
            RCLCPP_WARN(this->get_logger(), "Failed to enqueue message after making space.");
            return false;
        }
    }
    return true;
}

void OdometryProcessor::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    SerialMessage_t serial_msg;
    serial_msg.type = CMD_VEL;

    packFloatIntoArray(serial_msg.data, msg->linear.x, 0);
    packFloatIntoArray(serial_msg.data, msg->linear.y, sizeof(float));
    packFloatIntoArray(serial_msg.data, msg->angular.z, 2 * sizeof(float));

    if (!enqueueMessage(serial_msg)) {
        RCLCPP_WARN(this->get_logger(), "Unable to enqueue CMD_VEL message.");
    }
}

void OdometryProcessor::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    SerialMessage_t serial_msg;
    serial_msg.type = ODOMETRY;
    
    // Convert quaternion to Euler (yaw)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Pack position
    packFloatIntoArray(serial_msg.data, msg->pose.pose.position.x, 0);
    packFloatIntoArray(serial_msg.data, msg->pose.pose.position.y, 4);
    packFloatIntoArray(serial_msg.data, static_cast<float>(yaw), 8); // Packing yaw

    // Pack velocity
    packFloatIntoArray(serial_msg.data, msg->twist.twist.linear.x, 12);
    packFloatIntoArray(serial_msg.data, msg->twist.twist.linear.y, 16);
    packFloatIntoArray(serial_msg.data, msg->twist.twist.angular.z, 20);
    
    // Pack covariance
    packFloatIntoArray(serial_msg.data, msg->pose.covariance[0], 24);  // position_error.x
    packFloatIntoArray(serial_msg.data, msg->pose.covariance[7], 28);  // position_error.y
    packFloatIntoArray(serial_msg.data, msg->pose.covariance[35], 32); // position_error.angular
    packFloatIntoArray(serial_msg.data, msg->twist.covariance[0], 36);  // velocity_error.x
    packFloatIntoArray(serial_msg.data, msg->twist.covariance[7], 40);  // velocity_error.y
    packFloatIntoArray(serial_msg.data, msg->twist.covariance[35], 44); // velocity_error.angular
    
    if (!enqueueMessage(serial_msg)) {
        RCLCPP_WARN(this->get_logger(), "Unable to enqueue ODOMETRY message.");
    }
}

void OdometryProcessor::serialThread() {
    std::vector<uint8_t> read_buffer(SIZE_OF_RX_DATA, 0);

    while (rclcpp::ok()) {
        // Try to send one outgoing data message if available
        SerialMessage_t msg;
        if (message_queue_.pop(msg)) {
            sendSerialData(msg);
        }

        // Handle incoming data
        boost::system::error_code ec;
        size_t len = boost::asio::read(serial_, boost::asio::buffer(read_buffer), boost::asio::transfer_at_least(1), ec);
        
        if (!ec && len > 0) {
            processReceivedData(read_buffer);
        } else if (ec) {
            RCLCPP_WARN(this->get_logger(), "Error reading from serial: %s", ec.message().c_str());
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(10)); // Small sleep to prevent hogging CPU
    }
}


void OdometryProcessor::sendSerialData(const SerialMessage_t& msg) {
    std::array<uint8_t, SIZE_OF_TX_DATA> buffer;

    // Set headers
    buffer[0] = HEADER;
    buffer[1] = HEADER;

    // Set message type indicator
    buffer[2] = static_cast<uint8_t>(msg.type);

    // Copy data from SerialMessage into the buffer
    std::memcpy(&buffer[3], msg.data.data(), SIZE_OF_TX_DATA - 5); // SIZE_OF_TX_DATA - 5 to account for header, type, checksum, and tail

    // Compute checksum
    uint8_t checksum = 0;
    for (size_t i = 2; i < SIZE_OF_TX_DATA - 2; ++i) {
        checksum += buffer[i];
    }

    // Append checksum and tail
    buffer[SIZE_OF_TX_DATA - 2] = checksum;
    buffer[SIZE_OF_TX_DATA - 1] = TAIL;

    // Send data over serial port using boost::asio
    boost::asio::write(serial_, boost::asio::buffer(buffer, SIZE_OF_TX_DATA));
}

void OdometryProcessor::processReceivedData(const std::vector<uint8_t>& buffer) {
    if (buffer[0] == HEADER && buffer[1] == HEADER && buffer[SIZE_OF_RX_DATA - 1] == TAIL) {
        uint8_t checksum = 0;
        for (size_t i = 2; i < SIZE_OF_RX_DATA - 2; i++) {
            checksum += buffer[i];
        }
        uint8_t message_checksum = buffer[SIZE_OF_RX_DATA - 2];
        if (checksum == message_checksum) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->now();
            odom.header.frame_id = PARENT_FRAME_ID;

            memcpy(&odom.pose.pose.position.x, &buffer[2], sizeof(float));
            memcpy(&odom.pose.pose.position.y, &buffer[6], sizeof(float));
            
            // Assuming flat 2D odometry, interpreting angular as yaw
            float yaw;
            memcpy(&yaw, &buffer[10], sizeof(float));
            
            // Convert yaw (Euler angle) to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            memcpy(&odom.twist.twist.linear.x, &buffer[14], sizeof(float));
            memcpy(&odom.twist.twist.linear.y, &buffer[18], sizeof(float));
            memcpy(&odom.twist.twist.angular.z, &buffer[22], sizeof(float));

            // Covariance data
            memcpy(&odom.pose.covariance[0], &buffer[26], sizeof(float)); // position_error.x
            memcpy(&odom.pose.covariance[7], &buffer[30], sizeof(float)); // position_error.y
            memcpy(&odom.pose.covariance[35], &buffer[34], sizeof(float)); // position_error.angular

            memcpy(&odom.twist.covariance[0], &buffer[38], sizeof(float)); // velocity_error.x
            memcpy(&odom.twist.covariance[7], &buffer[42], sizeof(float)); // velocity_error.y
            memcpy(&odom.twist.covariance[35], &buffer[46], sizeof(float)); // velocity_error.angular

            odometry_pub_->publish(odom);
        } else {
            RCLCPP_WARN(this->get_logger(), "Checksum mismatch in received serial data.");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Header or Tail mismatch in received serial data.");
    }
}

void OdometryProcessor::spin() {
    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto processor = std::make_shared<OdometryProcessor>();
    processor->spin();
    rclcpp::shutdown();
    return 0;
}
