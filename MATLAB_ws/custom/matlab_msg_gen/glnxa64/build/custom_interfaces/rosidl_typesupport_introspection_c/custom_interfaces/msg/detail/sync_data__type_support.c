// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interfaces:msg/SyncData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interfaces/msg/detail/sync_data__rosidl_typesupport_introspection_c.h"
#include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interfaces/msg/detail/sync_data__functions.h"
#include "custom_interfaces/msg/detail/sync_data__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `ee_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `ee_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `measurements`
#include "std_msgs/msg/float32_multi_array.h"
// Member `measurements`
#include "std_msgs/msg/detail/float32_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__msg__SyncData__init(message_memory);
}

void custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_fini_function(void * message_memory)
{
  custom_interfaces__msg__SyncData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_member_array[3] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__SyncData, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ee_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__SyncData, ee_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "measurements",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__SyncData, measurements),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_members = {
  "custom_interfaces__msg",  // message namespace
  "SyncData",  // message name
  3,  // number of fields
  sizeof(custom_interfaces__msg__SyncData),
  false,  // has_any_key_member_
  custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_member_array,  // message members
  custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_type_support_handle = {
  0,
  &custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_members,
  get_message_typesupport_handle_function,
  &custom_interfaces__msg__SyncData__get_type_hash,
  &custom_interfaces__msg__SyncData__get_type_description,
  &custom_interfaces__msg__SyncData__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, msg, SyncData)() {
  custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32MultiArray)();
  if (!custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__msg__SyncData__rosidl_typesupport_introspection_c__SyncData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
