// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/BottleInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__BOTTLE_INFO__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__BOTTLE_INFO__BUILDER_HPP_

#include "interfaces/msg/detail/bottle_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_BottleInfo_position
{
public:
  explicit Init_BottleInfo_position(::interfaces::msg::BottleInfo & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::BottleInfo position(::interfaces::msg::BottleInfo::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::BottleInfo msg_;
};

class Init_BottleInfo_color
{
public:
  Init_BottleInfo_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BottleInfo_position color(::interfaces::msg::BottleInfo::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_BottleInfo_position(msg_);
  }

private:
  ::interfaces::msg::BottleInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::BottleInfo>()
{
  return interfaces::msg::builder::Init_BottleInfo_color();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__BOTTLE_INFO__BUILDER_HPP_
