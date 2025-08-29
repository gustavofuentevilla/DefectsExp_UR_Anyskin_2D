// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/SyncData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_interfaces/msg/sync_data.hpp"


#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'ee_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'measurements'
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__SyncData __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__SyncData __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SyncData_
{
  using Type = SyncData_<ContainerAllocator>;

  explicit SyncData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    ee_pose(_init),
    measurements(_init)
  {
    (void)_init;
  }

  explicit SyncData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    ee_pose(_alloc, _init),
    measurements(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _ee_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _ee_pose_type ee_pose;
  using _measurements_type =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator>;
  _measurements_type measurements;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__ee_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->ee_pose = _arg;
    return *this;
  }
  Type & set__measurements(
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> & _arg)
  {
    this->measurements = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::SyncData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::SyncData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::SyncData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::SyncData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__SyncData
    std::shared_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__SyncData
    std::shared_ptr<custom_interfaces::msg::SyncData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SyncData_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->ee_pose != other.ee_pose) {
      return false;
    }
    if (this->measurements != other.measurements) {
      return false;
    }
    return true;
  }
  bool operator!=(const SyncData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SyncData_

// alias to use template instance with default allocator
using SyncData =
  custom_interfaces::msg::SyncData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__STRUCT_HPP_
