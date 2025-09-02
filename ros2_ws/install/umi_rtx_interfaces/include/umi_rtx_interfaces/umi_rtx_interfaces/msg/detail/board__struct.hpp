// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__STRUCT_HPP_
#define UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__umi_rtx_interfaces__msg__Board __attribute__((deprecated))
#else
# define DEPRECATED__umi_rtx_interfaces__msg__Board __declspec(deprecated)
#endif

namespace umi_rtx_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Board_
{
  using Type = Board_<ContainerAllocator>;

  explicit Board_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 9>::iterator, int32_t>(this->data.begin(), this->data.end(), 0l);
    }
  }

  explicit Board_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 9>::iterator, int32_t>(this->data.begin(), this->data.end(), 0l);
    }
  }

  // field types and members
  using _data_type =
    std::array<int32_t, 9>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::array<int32_t, 9> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    umi_rtx_interfaces::msg::Board_<ContainerAllocator> *;
  using ConstRawPtr =
    const umi_rtx_interfaces::msg::Board_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      umi_rtx_interfaces::msg::Board_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      umi_rtx_interfaces::msg::Board_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__umi_rtx_interfaces__msg__Board
    std::shared_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__umi_rtx_interfaces__msg__Board
    std::shared_ptr<umi_rtx_interfaces::msg::Board_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Board_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Board_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Board_

// alias to use template instance with default allocator
using Board =
  umi_rtx_interfaces::msg::Board_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace umi_rtx_interfaces

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__STRUCT_HPP_
