// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/DetectionInfo.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/detection_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `text`
#include "rosidl_runtime_c/string_functions.h"
// Member `color`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
interfaces__msg__DetectionInfo__init(interfaces__msg__DetectionInfo * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    interfaces__msg__DetectionInfo__fini(msg);
    return false;
  }
  // x
  // y
  // z
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    interfaces__msg__DetectionInfo__fini(msg);
    return false;
  }
  // color
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->color, 0)) {
    interfaces__msg__DetectionInfo__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__DetectionInfo__fini(interfaces__msg__DetectionInfo * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // x
  // y
  // z
  // text
  rosidl_runtime_c__String__fini(&msg->text);
  // color
  rosidl_runtime_c__uint8__Sequence__fini(&msg->color);
}

bool
interfaces__msg__DetectionInfo__are_equal(const interfaces__msg__DetectionInfo * lhs, const interfaces__msg__DetectionInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  // color
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__DetectionInfo__copy(
  const interfaces__msg__DetectionInfo * input,
  interfaces__msg__DetectionInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  // color
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  return true;
}

interfaces__msg__DetectionInfo *
interfaces__msg__DetectionInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__DetectionInfo * msg = (interfaces__msg__DetectionInfo *)allocator.allocate(sizeof(interfaces__msg__DetectionInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__DetectionInfo));
  bool success = interfaces__msg__DetectionInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__DetectionInfo__destroy(interfaces__msg__DetectionInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__DetectionInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__DetectionInfo__Sequence__init(interfaces__msg__DetectionInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__DetectionInfo * data = NULL;

  if (size) {
    data = (interfaces__msg__DetectionInfo *)allocator.zero_allocate(size, sizeof(interfaces__msg__DetectionInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__DetectionInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__DetectionInfo__fini(&data[i - 1]);
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
interfaces__msg__DetectionInfo__Sequence__fini(interfaces__msg__DetectionInfo__Sequence * array)
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
      interfaces__msg__DetectionInfo__fini(&array->data[i]);
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

interfaces__msg__DetectionInfo__Sequence *
interfaces__msg__DetectionInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__DetectionInfo__Sequence * array = (interfaces__msg__DetectionInfo__Sequence *)allocator.allocate(sizeof(interfaces__msg__DetectionInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__DetectionInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__DetectionInfo__Sequence__destroy(interfaces__msg__DetectionInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__DetectionInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__DetectionInfo__Sequence__are_equal(const interfaces__msg__DetectionInfo__Sequence * lhs, const interfaces__msg__DetectionInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__DetectionInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__DetectionInfo__Sequence__copy(
  const interfaces__msg__DetectionInfo__Sequence * input,
  interfaces__msg__DetectionInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__DetectionInfo);
    interfaces__msg__DetectionInfo * data =
      (interfaces__msg__DetectionInfo *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__DetectionInfo__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interfaces__msg__DetectionInfo__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__DetectionInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
