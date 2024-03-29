// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/DetectionInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DETECTION_INFO__STRUCT_H_
#define INTERFACES__MSG__DETAIL__DETECTION_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'text'
#include "rosidl_runtime_c/string.h"
// Member 'color'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/DetectionInfo in the package interfaces.
typedef struct interfaces__msg__DetectionInfo
{
  double x;
  double y;
  rosidl_runtime_c__String text;
  rosidl_runtime_c__uint8__Sequence color;
} interfaces__msg__DetectionInfo;

// Struct for a sequence of interfaces__msg__DetectionInfo.
typedef struct interfaces__msg__DetectionInfo__Sequence
{
  interfaces__msg__DetectionInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__DetectionInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__DETECTION_INFO__STRUCT_H_
