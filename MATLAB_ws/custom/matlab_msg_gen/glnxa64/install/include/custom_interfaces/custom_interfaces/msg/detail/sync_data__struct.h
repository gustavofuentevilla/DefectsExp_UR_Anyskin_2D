// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/SyncData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_interfaces/msg/sync_data.h"


#ifndef CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'ee_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'measurements'
#include "std_msgs/msg/detail/float32_multi_array__struct.h"

/// Struct defined in msg/SyncData in the package custom_interfaces.
typedef struct custom_interfaces__msg__SyncData
{
  builtin_interfaces__msg__Time stamp;
  geometry_msgs__msg__PoseStamped ee_pose;
  std_msgs__msg__Float32MultiArray measurements;
} custom_interfaces__msg__SyncData;

// Struct for a sequence of custom_interfaces__msg__SyncData.
typedef struct custom_interfaces__msg__SyncData__Sequence
{
  custom_interfaces__msg__SyncData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__SyncData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__SYNC_DATA__STRUCT_H_
