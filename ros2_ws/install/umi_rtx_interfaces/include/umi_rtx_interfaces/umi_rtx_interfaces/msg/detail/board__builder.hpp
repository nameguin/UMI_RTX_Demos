// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__BUILDER_HPP_
#define UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "umi_rtx_interfaces/msg/detail/board__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace umi_rtx_interfaces
{

namespace msg
{

namespace builder
{

class Init_Board_data
{
public:
  Init_Board_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::umi_rtx_interfaces::msg::Board data(::umi_rtx_interfaces::msg::Board::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::Board msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::umi_rtx_interfaces::msg::Board>()
{
  return umi_rtx_interfaces::msg::builder::Init_Board_data();
}

}  // namespace umi_rtx_interfaces

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__BUILDER_HPP_
