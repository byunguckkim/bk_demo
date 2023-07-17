#pragma once

#if __has_include(<geometry_msgs/TransformStamped.h>)
// TODO: implement this for ros2

#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <tf2_msgs/TFMessage.h>

namespace applied::ros1 {

/**
 * @class BagWriter
 *
 * @brief Wrapper around a ROS bag to make it easier to save transforms and latched messages.
 */
class BagWriter {
 public:
  /**
   * @brief Constructor
   *
   * @param path Where the ROS bag will be written to.
   */
  explicit BagWriter(const std::string& path);

  BagWriter& operator=(BagWriter&& other) = delete;
  BagWriter(const BagWriter&) = delete;
  BagWriter& operator=(const BagWriter&) = delete;

  /**
   * @brief Attempts to open the ROS bag file for writing.
   *
   * @returns True if the ROS bag was opened successfully.
   */
  bool Open();

  /**
   * @brief Returns whether the bag is opened for writing.
   *
   * @returns True if the bag is open.
   */
  inline bool IsOpen() { return bag_.isOpen(); }

  /**
   * @brief Writes a ROS message to the bag.
   *
   * @param topic ROS topic to write the message to.
   * @param message ROS message object to write.
   * @param latched Whether the message should be latched (sent to future new subscribers).
   * @param time Bag timestamp used during write (not the time embedded in the message).
   */
  template <typename T>
  void Write(const std::string& topic, const T& msg, bool latched, const ros::Time& time) {
    if (!IsOpen()) {
      Open();
    }
    // There is no way to set latching other than defining a custom connection header.
    // We also need to add additional header fields as they would be inluded by default if we did
    // not specify a connection header.
    if (latched) {
      boost::shared_ptr<::ros::M_string> connection_header(new ::ros::M_string);
      (*connection_header)["latching"] = latched ? "1" : "0";
      (*connection_header)["type"] = std::string(::ros::message_traits::datatype(msg));
      (*connection_header)["md5sum"] = std::string(::ros::message_traits::md5sum(msg));
      (*connection_header)["msg_definition"] = std::string(::ros::message_traits::definition(msg));
      bag_.write(topic, time, msg, connection_header);
    } else {
      bag_.write(topic, time, msg);
    }
  }

  /**
   * @brief Write a stamped transform to the bag.
   *
   * @param transform Stamped transform object to write to the bag.
   * @param latched Whether the message should be latched (sent to future new subscribers).
   * @param time Bag timestamp used during write (not the time embedded in the message).
   */
  void WriteTransform(const geometry_msgs::TransformStamped& transform, bool latched,
                      const ros::Time& time);

  /**
   * @brief Write a vector of stamped transforms to the bag.
   *
   * @param transform Vector of transform objects to write to the bag.
   * @param latched Whether the message should be latched (sent to future new subscribers).
   * @param time Bag timestamp used during write (not the time embedded in the message).
   */
  void WriteTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms, bool latched,
                       const ros::Time& time);

 private:
  const std::string bag_path_;
  rosbag::Bag bag_;
};

}  // namespace applied::ros1

#endif  // __has_include(<geometry_msgs/TransformStamped.h>)
