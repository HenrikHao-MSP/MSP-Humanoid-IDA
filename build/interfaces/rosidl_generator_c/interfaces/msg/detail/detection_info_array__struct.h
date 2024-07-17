// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/DetectionInfoArray.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DETECTION_INFO_ARRAY__STRUCT_H_
#define INTERFACES__MSG__DETAIL__DETECTION_INFO_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'detections'
#include "interfaces/msg/detail/detection_info__struct.h"

// Struct defined in msg/DetectionInfoArray in the package interfaces.
typedef struct interfaces__msg__DetectionInfoArray
{
  interfaces__msg__DetectionInfo__Sequence detections;
} interfaces__msg__DetectionInfoArray;

// Struct for a sequence of interfaces__msg__DetectionInfoArray.
typedef struct interfaces__msg__DetectionInfoArray__Sequence
{
  interfaces__msg__DetectionInfoArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__DetectionInfoArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__DETECTION_INFO_ARRAY__STRUCT_H_
