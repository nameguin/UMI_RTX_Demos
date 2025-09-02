// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__BUILDER_HPP_
#define UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "umi_rtx_interfaces/msg/detail/game_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace umi_rtx_interfaces
{

namespace msg
{

namespace builder
{

class Init_GameData_isgamestarted
{
public:
  explicit Init_GameData_isgamestarted(::umi_rtx_interfaces::msg::GameData & msg)
  : msg_(msg)
  {}
  ::umi_rtx_interfaces::msg::GameData isgamestarted(::umi_rtx_interfaces::msg::GameData::_isgamestarted_type arg)
  {
    msg_.isgamestarted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::GameData msg_;
};

class Init_GameData_isrobotturn
{
public:
  explicit Init_GameData_isrobotturn(::umi_rtx_interfaces::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_isgamestarted isrobotturn(::umi_rtx_interfaces::msg::GameData::_isrobotturn_type arg)
  {
    msg_.isrobotturn = std::move(arg);
    return Init_GameData_isgamestarted(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::GameData msg_;
};

class Init_GameData_secondarymsg
{
public:
  explicit Init_GameData_secondarymsg(::umi_rtx_interfaces::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_isrobotturn secondarymsg(::umi_rtx_interfaces::msg::GameData::_secondarymsg_type arg)
  {
    msg_.secondarymsg = std::move(arg);
    return Init_GameData_isrobotturn(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::GameData msg_;
};

class Init_GameData_primarymsg
{
public:
  explicit Init_GameData_primarymsg(::umi_rtx_interfaces::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_secondarymsg primarymsg(::umi_rtx_interfaces::msg::GameData::_primarymsg_type arg)
  {
    msg_.primarymsg = std::move(arg);
    return Init_GameData_secondarymsg(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::GameData msg_;
};

class Init_GameData_moveshistory
{
public:
  explicit Init_GameData_moveshistory(::umi_rtx_interfaces::msg::GameData & msg)
  : msg_(msg)
  {}
  Init_GameData_primarymsg moveshistory(::umi_rtx_interfaces::msg::GameData::_moveshistory_type arg)
  {
    msg_.moveshistory = std::move(arg);
    return Init_GameData_primarymsg(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::GameData msg_;
};

class Init_GameData_board
{
public:
  Init_GameData_board()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameData_moveshistory board(::umi_rtx_interfaces::msg::GameData::_board_type arg)
  {
    msg_.board = std::move(arg);
    return Init_GameData_moveshistory(msg_);
  }

private:
  ::umi_rtx_interfaces::msg::GameData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::umi_rtx_interfaces::msg::GameData>()
{
  return umi_rtx_interfaces::msg::builder::Init_GameData_board();
}

}  // namespace umi_rtx_interfaces

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__BUILDER_HPP_
