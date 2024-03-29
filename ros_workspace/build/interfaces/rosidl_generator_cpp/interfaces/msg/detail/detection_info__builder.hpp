// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/DetectionInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DETECTION_INFO__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__DETECTION_INFO__BUILDER_HPP_

#include "interfaces/msg/detail/detection_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectionInfo_color
{
public:
  explicit Init_DetectionInfo_color(::interfaces::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::DetectionInfo color(::interfaces::msg::DetectionInfo::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_text
{
public:
  explicit Init_DetectionInfo_text(::interfaces::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  Init_DetectionInfo_color text(::interfaces::msg::DetectionInfo::_text_type arg)
  {
    msg_.text = std::move(arg);
    return Init_DetectionInfo_color(msg_);
  }

private:
  ::interfaces::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_z
{
public:
  explicit Init_DetectionInfo_z(::interfaces::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  Init_DetectionInfo_text z(::interfaces::msg::DetectionInfo::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_DetectionInfo_text(msg_);
  }

private:
  ::interfaces::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_y
{
public:
  explicit Init_DetectionInfo_y(::interfaces::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  Init_DetectionInfo_z y(::interfaces::msg::DetectionInfo::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DetectionInfo_z(msg_);
  }

private:
  ::interfaces::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_x
{
public:
  explicit Init_DetectionInfo_x(::interfaces::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  Init_DetectionInfo_y x(::interfaces::msg::DetectionInfo::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DetectionInfo_y(msg_);
  }

private:
  ::interfaces::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_name
{
public:
  Init_DetectionInfo_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionInfo_x name(::interfaces::msg::DetectionInfo::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_DetectionInfo_x(msg_);
  }

private:
  ::interfaces::msg::DetectionInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::DetectionInfo>()
{
  return interfaces::msg::builder::Init_DetectionInfo_name();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__DETECTION_INFO__BUILDER_HPP_
