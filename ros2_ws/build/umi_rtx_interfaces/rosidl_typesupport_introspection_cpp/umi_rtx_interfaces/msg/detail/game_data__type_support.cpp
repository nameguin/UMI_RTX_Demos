// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "umi_rtx_interfaces/msg/detail/game_data__functions.h"
#include "umi_rtx_interfaces/msg/detail/game_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace umi_rtx_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GameData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) umi_rtx_interfaces::msg::GameData(_init);
}

void GameData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<umi_rtx_interfaces::msg::GameData *>(message_memory);
  typed_message->~GameData();
}

size_t size_function__GameData__moveshistory(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__GameData__moveshistory(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<std::string, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__GameData__moveshistory(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<std::string, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__GameData__moveshistory(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GameData__moveshistory(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GameData__moveshistory(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GameData__moveshistory(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GameData_message_member_array[6] = {
  {
    "board",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<umi_rtx_interfaces::msg::Board>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces::msg::GameData, board),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "moveshistory",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces::msg::GameData, moveshistory),  // bytes offset in struct
    nullptr,  // default value
    size_function__GameData__moveshistory,  // size() function pointer
    get_const_function__GameData__moveshistory,  // get_const(index) function pointer
    get_function__GameData__moveshistory,  // get(index) function pointer
    fetch_function__GameData__moveshistory,  // fetch(index, &value) function pointer
    assign_function__GameData__moveshistory,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "primarymsg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces::msg::GameData, primarymsg),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "secondarymsg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces::msg::GameData, secondarymsg),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "isrobotturn",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces::msg::GameData, isrobotturn),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "isgamestarted",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces::msg::GameData, isgamestarted),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GameData_message_members = {
  "umi_rtx_interfaces::msg",  // message namespace
  "GameData",  // message name
  6,  // number of fields
  sizeof(umi_rtx_interfaces::msg::GameData),
  GameData_message_member_array,  // message members
  GameData_init_function,  // function to initialize message memory (memory has to be allocated)
  GameData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GameData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GameData_message_members,
  get_message_typesupport_handle_function,
  &umi_rtx_interfaces__msg__GameData__get_type_hash,
  &umi_rtx_interfaces__msg__GameData__get_type_description,
  &umi_rtx_interfaces__msg__GameData__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace umi_rtx_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<umi_rtx_interfaces::msg::GameData>()
{
  return &::umi_rtx_interfaces::msg::rosidl_typesupport_introspection_cpp::GameData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, umi_rtx_interfaces, msg, GameData)() {
  return &::umi_rtx_interfaces::msg::rosidl_typesupport_introspection_cpp::GameData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
