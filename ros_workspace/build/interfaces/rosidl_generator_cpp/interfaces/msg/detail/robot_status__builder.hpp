// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include "interfaces/msg/detail/robot_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_status
{
public:
  Init_RobotStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::RobotStatus status(::interfaces::msg::RobotStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::RobotStatus>()
{
  return interfaces::msg::builder::Init_RobotStatus_status();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
