// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/DetectionInfoArray.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/detection_info_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `detections`
#include "interfaces/msg/detail/detection_info__functions.h"

bool
interfaces__msg__DetectionInfoArray__init(interfaces__msg__DetectionInfoArray * msg)
{
  if (!msg) {
    return false;
  }
  // detections
  if (!interfaces__msg__DetectionInfo__Sequence__init(&msg->detections, 0)) {
    interfaces__msg__DetectionInfoArray__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__DetectionInfoArray__fini(interfaces__msg__DetectionInfoArray * msg)
{
  if (!msg) {
    return;
  }
  // detections
  interfaces__msg__DetectionInfo__Sequence__fini(&msg->detections);
}

bool
interfaces__msg__DetectionInfoArray__are_equal(const interfaces__msg__DetectionInfoArray * lhs, const interfaces__msg__DetectionInfoArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // detections
  if (!interfaces__msg__DetectionInfo__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__DetectionInfoArray__copy(
  const interfaces__msg__DetectionInfoArray * input,
  interfaces__msg__DetectionInfoArray * output)
{
  if (!input || !output) {
    return false;
  }
  // detections
  if (!interfaces__msg__DetectionInfo__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

interfaces__msg__DetectionInfoArray *
interfaces__msg__DetectionInfoArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__DetectionInfoArray * msg = (interfaces__msg__DetectionInfoArray *)allocator.allocate(sizeof(interfaces__msg__DetectionInfoArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__DetectionInfoArray));
  bool success = interfaces__msg__DetectionInfoArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__DetectionInfoArray__destroy(interfaces__msg__DetectionInfoArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__DetectionInfoArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__DetectionInfoArray__Sequence__init(interfaces__msg__DetectionInfoArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__DetectionInfoArray * data = NULL;

  if (size) {
    data = (interfaces__msg__DetectionInfoArray *)allocator.zero_allocate(size, sizeof(interfaces__msg__DetectionInfoArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__DetectionInfoArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__DetectionInfoArray__fini(&data[i - 1]);
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
interfaces__msg__DetectionInfoArray__Sequence__fini(interfaces__msg__DetectionInfoArray__Sequence * array)
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
      interfaces__msg__DetectionInfoArray__fini(&array->data[i]);
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

interfaces__msg__DetectionInfoArray__Sequence *
interfaces__msg__DetectionInfoArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__DetectionInfoArray__Sequence * array = (interfaces__msg__DetectionInfoArray__Sequence *)allocator.allocate(sizeof(interfaces__msg__DetectionInfoArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__DetectionInfoArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__DetectionInfoArray__Sequence__destroy(interfaces__msg__DetectionInfoArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__DetectionInfoArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__DetectionInfoArray__Sequence__are_equal(const interfaces__msg__DetectionInfoArray__Sequence * lhs, const interfaces__msg__DetectionInfoArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__DetectionInfoArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__DetectionInfoArray__Sequence__copy(
  const interfaces__msg__DetectionInfoArray__Sequence * input,
  interfaces__msg__DetectionInfoArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__DetectionInfoArray);
    interfaces__msg__DetectionInfoArray * data =
      (interfaces__msg__DetectionInfoArray *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__DetectionInfoArray__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interfaces__msg__DetectionInfoArray__fini(&data[i]);
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
    if (!interfaces__msg__DetectionInfoArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
