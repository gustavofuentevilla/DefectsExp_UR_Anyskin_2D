// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/SyncData.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/sync_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `ee_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `measurements`
#include "std_msgs/msg/detail/float32_multi_array__functions.h"

bool
custom_interfaces__msg__SyncData__init(custom_interfaces__msg__SyncData * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    custom_interfaces__msg__SyncData__fini(msg);
    return false;
  }
  // ee_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->ee_pose)) {
    custom_interfaces__msg__SyncData__fini(msg);
    return false;
  }
  // measurements
  if (!std_msgs__msg__Float32MultiArray__init(&msg->measurements)) {
    custom_interfaces__msg__SyncData__fini(msg);
    return false;
  }
  return true;
}

void
custom_interfaces__msg__SyncData__fini(custom_interfaces__msg__SyncData * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // ee_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->ee_pose);
  // measurements
  std_msgs__msg__Float32MultiArray__fini(&msg->measurements);
}

bool
custom_interfaces__msg__SyncData__are_equal(const custom_interfaces__msg__SyncData * lhs, const custom_interfaces__msg__SyncData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // ee_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->ee_pose), &(rhs->ee_pose)))
  {
    return false;
  }
  // measurements
  if (!std_msgs__msg__Float32MultiArray__are_equal(
      &(lhs->measurements), &(rhs->measurements)))
  {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__SyncData__copy(
  const custom_interfaces__msg__SyncData * input,
  custom_interfaces__msg__SyncData * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // ee_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->ee_pose), &(output->ee_pose)))
  {
    return false;
  }
  // measurements
  if (!std_msgs__msg__Float32MultiArray__copy(
      &(input->measurements), &(output->measurements)))
  {
    return false;
  }
  return true;
}

custom_interfaces__msg__SyncData *
custom_interfaces__msg__SyncData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__SyncData * msg = (custom_interfaces__msg__SyncData *)allocator.allocate(sizeof(custom_interfaces__msg__SyncData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__SyncData));
  bool success = custom_interfaces__msg__SyncData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__SyncData__destroy(custom_interfaces__msg__SyncData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__SyncData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__SyncData__Sequence__init(custom_interfaces__msg__SyncData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__SyncData * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__SyncData *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__SyncData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__SyncData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__SyncData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interfaces__msg__SyncData__Sequence__fini(custom_interfaces__msg__SyncData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interfaces__msg__SyncData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interfaces__msg__SyncData__Sequence *
custom_interfaces__msg__SyncData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__SyncData__Sequence * array = (custom_interfaces__msg__SyncData__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__SyncData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__SyncData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__SyncData__Sequence__destroy(custom_interfaces__msg__SyncData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__SyncData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__SyncData__Sequence__are_equal(const custom_interfaces__msg__SyncData__Sequence * lhs, const custom_interfaces__msg__SyncData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__SyncData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__SyncData__Sequence__copy(
  const custom_interfaces__msg__SyncData__Sequence * input,
  custom_interfaces__msg__SyncData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__SyncData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__SyncData * data =
      (custom_interfaces__msg__SyncData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__SyncData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__SyncData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__SyncData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
