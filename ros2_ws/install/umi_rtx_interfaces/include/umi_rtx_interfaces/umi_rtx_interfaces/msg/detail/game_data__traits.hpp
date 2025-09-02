// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__TRAITS_HPP_
#define UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "umi_rtx_interfaces/msg/detail/game_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'board'
#include "umi_rtx_interfaces/msg/detail/board__traits.hpp"

namespace umi_rtx_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GameData & msg,
  std::ostream & out)
{
  out << "{";
  // member: board
  {
    out << "board: ";
    to_flow_style_yaml(msg.board, out);
    out << ", ";
  }

  // member: moveshistory
  {
    if (msg.moveshistory.size() == 0) {
      out << "moveshistory: []";
    } else {
      out << "moveshistory: [";
      size_t pending_items = msg.moveshistory.size();
      for (auto item : msg.moveshistory) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: primarymsg
  {
    out << "primarymsg: ";
    rosidl_generator_traits::value_to_yaml(msg.primarymsg, out);
    out << ", ";
  }

  // member: secondarymsg
  {
    out << "secondarymsg: ";
    rosidl_generator_traits::value_to_yaml(msg.secondarymsg, out);
    out << ", ";
  }

  // member: isrobotturn
  {
    out << "isrobotturn: ";
    rosidl_generator_traits::value_to_yaml(msg.isrobotturn, out);
    out << ", ";
  }

  // member: isgamestarted
  {
    out << "isgamestarted: ";
    rosidl_generator_traits::value_to_yaml(msg.isgamestarted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GameData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: board
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board:\n";
    to_block_style_yaml(msg.board, out, indentation + 2);
  }

  // member: moveshistory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.moveshistory.size() == 0) {
      out << "moveshistory: []\n";
    } else {
      out << "moveshistory:\n";
      for (auto item : msg.moveshistory) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: primarymsg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "primarymsg: ";
    rosidl_generator_traits::value_to_yaml(msg.primarymsg, out);
    out << "\n";
  }

  // member: secondarymsg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "secondarymsg: ";
    rosidl_generator_traits::value_to_yaml(msg.secondarymsg, out);
    out << "\n";
  }

  // member: isrobotturn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "isrobotturn: ";
    rosidl_generator_traits::value_to_yaml(msg.isrobotturn, out);
    out << "\n";
  }

  // member: isgamestarted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "isgamestarted: ";
    rosidl_generator_traits::value_to_yaml(msg.isgamestarted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GameData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace umi_rtx_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use umi_rtx_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const umi_rtx_interfaces::msg::GameData & msg,
  std::ostream & out, size_t indentation = 0)
{
  umi_rtx_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use umi_rtx_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const umi_rtx_interfaces::msg::GameData & msg)
{
  return umi_rtx_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<umi_rtx_interfaces::msg::GameData>()
{
  return "umi_rtx_interfaces::msg::GameData";
}

template<>
inline const char * name<umi_rtx_interfaces::msg::GameData>()
{
  return "umi_rtx_interfaces/msg/GameData";
}

template<>
struct has_fixed_size<umi_rtx_interfaces::msg::GameData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<umi_rtx_interfaces::msg::GameData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<umi_rtx_interfaces::msg::GameData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__TRAITS_HPP_
