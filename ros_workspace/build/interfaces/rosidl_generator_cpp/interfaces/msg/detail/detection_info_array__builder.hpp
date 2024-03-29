// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/DetectionInfoArray.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DETECTION_INFO_ARRAY__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__DETECTION_INFO_ARRAY__BUILDER_HPP_

#include "interfaces/msg/detail/detection_info_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectionInfoArray_detections
{
public:
  Init_DetectionInfoArray_detections()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::DetectionInfoArray detections(::interfaces::msg::DetectionInfoArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::DetectionInfoArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::DetectionInfoArray>()
{
  return interfaces::msg::builder::Init_DetectionInfoArray_detections();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__DETECTION_INFO_ARRAY__BUILDER_HPP_
