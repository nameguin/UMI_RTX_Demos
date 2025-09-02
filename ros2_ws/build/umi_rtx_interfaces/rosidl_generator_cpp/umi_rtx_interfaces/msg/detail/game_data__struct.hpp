// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__STRUCT_HPP_
#define UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'board'
#include "umi_rtx_interfaces/msg/detail/board__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__umi_rtx_interfaces__msg__GameData __attribute__((deprecated))
#else
# define DEPRECATED__umi_rtx_interfaces__msg__GameData __declspec(deprecated)
#endif

namespace umi_rtx_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GameData_
{
  using Type = GameData_<ContainerAllocator>;

  explicit GameData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : board(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 9>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->moveshistory.begin(), this->moveshistory.end(), "");
      this->primarymsg = "";
      this->secondarymsg = "";
      this->isrobotturn = false;
      this->isgamestarted = false;
    }
  }

  explicit GameData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : board(_alloc, _init),
    moveshistory(_alloc),
    primarymsg(_alloc),
    secondarymsg(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 9>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->moveshistory.begin(), this->moveshistory.end(), "");
      this->primarymsg = "";
      this->secondarymsg = "";
      this->isrobotturn = false;
      this->isgamestarted = false;
    }
  }

  // field types and members
  using _board_type =
    umi_rtx_interfaces::msg::Board_<ContainerAllocator>;
  _board_type board;
  using _moveshistory_type =
    std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 9>;
  _moveshistory_type moveshistory;
  using _primarymsg_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _primarymsg_type primarymsg;
  using _secondarymsg_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _secondarymsg_type secondarymsg;
  using _isrobotturn_type =
    bool;
  _isrobotturn_type isrobotturn;
  using _isgamestarted_type =
    bool;
  _isgamestarted_type isgamestarted;

  // setters for named parameter idiom
  Type & set__board(
    const umi_rtx_interfaces::msg::Board_<ContainerAllocator> & _arg)
  {
    this->board = _arg;
    return *this;
  }
  Type & set__moveshistory(
    const std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 9> & _arg)
  {
    this->moveshistory = _arg;
    return *this;
  }
  Type & set__primarymsg(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->primarymsg = _arg;
    return *this;
  }
  Type & set__secondarymsg(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->secondarymsg = _arg;
    return *this;
  }
  Type & set__isrobotturn(
    const bool & _arg)
  {
    this->isrobotturn = _arg;
    return *this;
  }
  Type & set__isgamestarted(
    const bool & _arg)
  {
    this->isgamestarted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    umi_rtx_interfaces::msg::GameData_<ContainerAllocator> *;
  using ConstRawPtr =
    const umi_rtx_interfaces::msg::GameData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      umi_rtx_interfaces::msg::GameData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      umi_rtx_interfaces::msg::GameData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__umi_rtx_interfaces__msg__GameData
    std::shared_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__umi_rtx_interfaces__msg__GameData
    std::shared_ptr<umi_rtx_interfaces::msg::GameData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GameData_ & other) const
  {
    if (this->board != other.board) {
      return false;
    }
    if (this->moveshistory != other.moveshistory) {
      return false;
    }
    if (this->primarymsg != other.primarymsg) {
      return false;
    }
    if (this->secondarymsg != other.secondarymsg) {
      return false;
    }
    if (this->isrobotturn != other.isrobotturn) {
      return false;
    }
    if (this->isgamestarted != other.isgamestarted) {
      return false;
    }
    return true;
  }
  bool operator!=(const GameData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GameData_

// alias to use template instance with default allocator
using GameData =
  umi_rtx_interfaces::msg::GameData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace umi_rtx_interfaces

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__STRUCT_HPP_
