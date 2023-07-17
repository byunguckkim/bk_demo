#include "applied/simian/public/ros_helpers/adapters.h"

namespace applied::ros::adapters {

geometry_msgs_ns::Vector3 AdaptVector3(const simian_public::spatial::Pose& pose) {
  geometry_msgs_ns::Vector3 vector;
  vector.x = pose.px();
  vector.y = pose.py();
  vector.z = pose.pz();
  return vector;
}

geometry_msgs_ns::Point AdaptPoint(const simian_public::spatial::Pose& pose) {
  geometry_msgs_ns::Point point;
  point.x = pose.px();
  point.y = pose.py();
  point.z = pose.pz();
  return point;
}

geometry_msgs_ns::Quaternion AdaptQuaternion(const simian_public::spatial::Pose& pose) {
  geometry_msgs_ns::Quaternion quaternion;
  quaternion.x = pose.qx();
  quaternion.y = pose.qy();
  quaternion.z = pose.qz();
  quaternion.w = pose.qw();
  return quaternion;
}

time_ns::Time AdaptRosTime(const google::protobuf::Timestamp& sim_time) {
  return {static_cast<ros_seconds_t>(sim_time.seconds()), static_cast<uint32_t>(sim_time.nanos())};
}

geometry_msgs_ns::TransformStamped AdaptStampedTransform(
    std::string_view parent_frame_id, std::string_view child_frame_id,
    const google::protobuf::Timestamp& sim_time, const simian_public::spatial::Pose& pose) {
  geometry_msgs_ns::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.header.stamp = AdaptRosTime(sim_time);
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = AdaptVector3(pose);
  transform_stamped.transform.rotation = AdaptQuaternion(pose);
  return transform_stamped;
}

geometry_msgs_ns::TransformStamped AdaptEgoTransform(const google::protobuf::Timestamp& sim_time,
                                                     const simian_public::spatial::Pose& ego_pose) {
  return AdaptStampedTransform("map", "ego", sim_time, ego_pose);
}

geometry_msgs_ns::PoseStamped AdaptStampedPose(std::string_view parent_frame_id,
                                               const google::protobuf::Timestamp& sim_time,
                                               const simian_public::spatial::Pose& pose) {
  geometry_msgs_ns::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = parent_frame_id;
  pose_stamped.header.stamp = AdaptRosTime(sim_time);
  pose_stamped.pose.position = AdaptPoint(pose);
  pose_stamped.pose.orientation = AdaptQuaternion(pose);
  return pose_stamped;
}

sensor_msgs_ns::Imu AdaptEgoImu(const google::protobuf::Timestamp& sim_time,
                                const simian_public::ego::EgoOutput::Section& ego_section) {
  const simian_public::spatial::State& state = ego_section.state();
  sensor_msgs_ns::Imu imu;
  imu.header.frame_id = "ego";
  imu.header.stamp = AdaptRosTime(sim_time);
  imu.orientation = AdaptQuaternion(state.pose());
  imu.orientation_covariance.fill(0.0f);
  imu.angular_velocity.x = state.velocity().rx();
  imu.angular_velocity.y = state.velocity().ry();
  imu.angular_velocity.z = state.velocity().rz();
  imu.angular_velocity_covariance.fill(0.0f);
  imu.linear_acceleration.x = state.acceleration().tx();
  imu.linear_acceleration.y = state.acceleration().ty();
  imu.linear_acceleration.z = state.acceleration().tz();
  return imu;
}

std_msgs_ns::ColorRGBA DefaultMarkerColor() {
  std_msgs_ns::ColorRGBA color;
  color.g = 1.0;
  color.a = 0.3;
  return color;
}

visualization_msgs_ns::Marker AdaptActorMarker(const google::protobuf::Timestamp& sim_time,
                                               const simian_public::actor::Actor& actor,
                                               const std_msgs_ns::ColorRGBA& marker_color) {
  visualization_msgs_ns::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = AdaptRosTime(sim_time);
  marker.id = actor.id();
  marker.pose.position.x = actor.pose().px();
  marker.pose.position.y = actor.pose().py();
  marker.pose.position.z = actor.pose().pz();
  marker.pose.orientation = AdaptQuaternion(actor.pose());
  marker.scale.x = actor.legacy().length();
  marker.scale.y = actor.legacy().width();
  marker.scale.z = actor.legacy().height();
  marker.lifetime = time_ns::Duration(30, 0);
  marker.type = visualization_msgs_ns::Marker::CUBE;
  marker.color = marker_color;
  // Simian pose is where the the rubber meets the road.
  // ROS expects the z position to be the center of the object.
  marker.pose.position.z += marker.scale.z / 2.0;
  return marker;
}

visualization_msgs_ns::MarkerArray AdaptActorMarkers(
    const google::protobuf::Timestamp& sim_time,
    const simian_public::perception::PerceptionChannel::ActorSensor& actor_sensor,
    const std_msgs_ns::ColorRGBA& marker_color) {
  visualization_msgs_ns::MarkerArray marker_array;
  for (const auto& actor : actor_sensor.actors()) {
    marker_array.markers.push_back(AdaptActorMarker(sim_time, actor, marker_color));
  }
  return marker_array;
}

}  // namespace applied::ros::adapters
