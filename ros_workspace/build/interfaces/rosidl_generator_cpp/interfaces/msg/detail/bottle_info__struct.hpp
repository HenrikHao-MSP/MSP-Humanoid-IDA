// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/BottleInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__BOTTLE_INFO__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__BOTTLE_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__BottleInfo __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__BottleInfo __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BottleInfo_
{
  using Type = BottleInfo_<ContainerAllocator>;

  explicit BottleInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color = "";
      this->position = "";
    }
  }

  explicit BottleInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc),
    position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color = "";
      this->position = "";
    }
  }

  // field types and members
  using _color_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _color_type color;
  using _position_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__color(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__position(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::BottleInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::BottleInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::BottleInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::BottleInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::BottleInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::BottleInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::BottleInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::BottleInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::BottleInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::BottleInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__BottleInfo
    std::shared_ptr<interfaces::msg::BottleInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__BottleInfo
    std::shared_ptr<interfaces::msg::BottleInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BottleInfo_ & other) const
  {
    if (this->color != other.color) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const BottleInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BottleInfo_

// alias to use template instance with default allocator
using BottleInfo =
  interfaces::msg::BottleInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__BOTTLE_INFO__STRUCT_HPP_
