// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/SyncData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_interfaces/msg/sync_data.hpp"


#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/sync_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'ee_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'measurements'
#include "std_msgs/msg/detail/float32_multi_array__traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SyncData & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: ee_pose
  {
    out << "ee_pose: ";
    to_flow_style_yaml(msg.ee_pose, out);
    out << ", ";
  }

  // member: measurements
  {
    out << "measurements: ";
    to_flow_style_yaml(msg.measurements, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SyncData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: ee_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ee_pose:\n";
    to_block_style_yaml(msg.ee_pose, out, indentation + 2);
  }

  // member: measurements
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measurements:\n";
    to_block_style_yaml(msg.measurements, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SyncData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::msg::SyncData & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::SyncData & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::SyncData>()
{
  return "custom_interfaces::msg::SyncData";
}

template<>
inline const char * name<custom_interfaces::msg::SyncData>()
{
  return "custom_interfaces/msg/SyncData";
}

template<>
struct has_fixed_size<custom_interfaces::msg::SyncData>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<geometry_msgs::msg::PoseStamped>::value && has_fixed_size<std_msgs::msg::Float32MultiArray>::value> {};

template<>
struct has_bounded_size<custom_interfaces::msg::SyncData>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<geometry_msgs::msg::PoseStamped>::value && has_bounded_size<std_msgs::msg::Float32MultiArray>::value> {};

template<>
struct is_message<custom_interfaces::msg::SyncData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__TRAITS_HPP_
