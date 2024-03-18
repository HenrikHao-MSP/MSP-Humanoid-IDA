// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_

#include "interfaces/msg/detail/robot_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::msg::RobotStatus>()
{
  return "interfaces::msg::RobotStatus";
}

template<>
inline const char * name<interfaces::msg::RobotStatus>()
{
  return "interfaces/msg/RobotStatus";
}

template<>
struct has_fixed_size<interfaces::msg::RobotStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::RobotStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::RobotStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
