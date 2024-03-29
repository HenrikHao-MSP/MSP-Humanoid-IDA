// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interfaces:msg/DetectionInfoArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interfaces/msg/detail/detection_info_array__rosidl_typesupport_introspection_c.h"
#include "interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interfaces/msg/detail/detection_info_array__functions.h"
#include "interfaces/msg/detail/detection_info_array__struct.h"


// Include directives for member types
// Member `detections`
#include "interfaces/msg/detection_info.h"
// Member `detections`
#include "interfaces/msg/detail/detection_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interfaces__msg__DetectionInfoArray__init(message_memory);
}

void DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_fini_function(void * message_memory)
{
  interfaces__msg__DetectionInfoArray__fini(message_memory);
}

size_t DetectionInfoArray__rosidl_typesupport_introspection_c__size_function__DetectionInfo__detections(
  const void * untyped_member)
{
  const interfaces__msg__DetectionInfo__Sequence * member =
    (const interfaces__msg__DetectionInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * DetectionInfoArray__rosidl_typesupport_introspection_c__get_const_function__DetectionInfo__detections(
  const void * untyped_member, size_t index)
{
  const interfaces__msg__DetectionInfo__Sequence * member =
    (const interfaces__msg__DetectionInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * DetectionInfoArray__rosidl_typesupport_introspection_c__get_function__DetectionInfo__detections(
  void * untyped_member, size_t index)
{
  interfaces__msg__DetectionInfo__Sequence * member =
    (interfaces__msg__DetectionInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

bool DetectionInfoArray__rosidl_typesupport_introspection_c__resize_function__DetectionInfo__detections(
  void * untyped_member, size_t size)
{
  interfaces__msg__DetectionInfo__Sequence * member =
    (interfaces__msg__DetectionInfo__Sequence *)(untyped_member);
  interfaces__msg__DetectionInfo__Sequence__fini(member);
  return interfaces__msg__DetectionInfo__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_member_array[1] = {
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__DetectionInfoArray, detections),  // bytes offset in struct
    NULL,  // default value
    DetectionInfoArray__rosidl_typesupport_introspection_c__size_function__DetectionInfo__detections,  // size() function pointer
    DetectionInfoArray__rosidl_typesupport_introspection_c__get_const_function__DetectionInfo__detections,  // get_const(index) function pointer
    DetectionInfoArray__rosidl_typesupport_introspection_c__get_function__DetectionInfo__detections,  // get(index) function pointer
    DetectionInfoArray__rosidl_typesupport_introspection_c__resize_function__DetectionInfo__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_members = {
  "interfaces__msg",  // message namespace
  "DetectionInfoArray",  // message name
  1,  // number of fields
  sizeof(interfaces__msg__DetectionInfoArray),
  DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_member_array,  // message members
  DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_type_support_handle = {
  0,
  &DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces, msg, DetectionInfoArray)() {
  DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces, msg, DetectionInfo)();
  if (!DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_type_support_handle.typesupport_identifier) {
    DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DetectionInfoArray__rosidl_typesupport_introspection_c__DetectionInfoArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
