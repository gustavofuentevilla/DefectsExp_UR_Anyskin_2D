// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/SyncData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_interfaces/msg/sync_data.hpp"


#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/sync_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_SyncData_measurements
{
public:
  explicit Init_SyncData_measurements(::custom_interfaces::msg::SyncData & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::SyncData measurements(::custom_interfaces::msg::SyncData::_measurements_type arg)
  {
    msg_.measurements = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::SyncData msg_;
};

class Init_SyncData_ee_pose
{
public:
  explicit Init_SyncData_ee_pose(::custom_interfaces::msg::SyncData & msg)
  : msg_(msg)
  {}
  Init_SyncData_measurements ee_pose(::custom_interfaces::msg::SyncData::_ee_pose_type arg)
  {
    msg_.ee_pose = std::move(arg);
    return Init_SyncData_measurements(msg_);
  }

private:
  ::custom_interfaces::msg::SyncData msg_;
};

class Init_SyncData_stamp
{
public:
  Init_SyncData_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SyncData_ee_pose stamp(::custom_interfaces::msg::SyncData::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_SyncData_ee_pose(msg_);
  }

private:
  ::custom_interfaces::msg::SyncData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::SyncData>()
{
  return custom_interfaces::msg::builder::Init_SyncData_stamp();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__BUILDER_HPP_
