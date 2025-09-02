// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "umi_rtx_interfaces/msg/detail/game_data__rosidl_typesupport_introspection_c.h"
#include "umi_rtx_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "umi_rtx_interfaces/msg/detail/game_data__functions.h"
#include "umi_rtx_interfaces/msg/detail/game_data__struct.h"


// Include directives for member types
// Member `board`
#include "umi_rtx_interfaces/msg/board.h"
// Member `board`
#include "umi_rtx_interfaces/msg/detail/board__rosidl_typesupport_introspection_c.h"
// Member `moveshistory`
// Member `primarymsg`
// Member `secondarymsg`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  umi_rtx_interfaces__msg__GameData__init(message_memory);
}

void umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_fini_function(void * message_memory)
{
  umi_rtx_interfaces__msg__GameData__fini(message_memory);
}

size_t umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__size_function__GameData__moveshistory(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__get_const_function__GameData__moveshistory(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__get_function__GameData__moveshistory(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__fetch_function__GameData__moveshistory(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__get_const_function__GameData__moveshistory(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__assign_function__GameData__moveshistory(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__get_function__GameData__moveshistory(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_member_array[6] = {
  {
    "board",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__GameData, board),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "moveshistory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__GameData, moveshistory),  // bytes offset in struct
    NULL,  // default value
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__size_function__GameData__moveshistory,  // size() function pointer
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__get_const_function__GameData__moveshistory,  // get_const(index) function pointer
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__get_function__GameData__moveshistory,  // get(index) function pointer
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__fetch_function__GameData__moveshistory,  // fetch(index, &value) function pointer
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__assign_function__GameData__moveshistory,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "primarymsg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__GameData, primarymsg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "secondarymsg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__GameData, secondarymsg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "isrobotturn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__GameData, isrobotturn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "isgamestarted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__GameData, isgamestarted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_members = {
  "umi_rtx_interfaces__msg",  // message namespace
  "GameData",  // message name
  6,  // number of fields
  sizeof(umi_rtx_interfaces__msg__GameData),
  umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_member_array,  // message members
  umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_init_function,  // function to initialize message memory (memory has to be allocated)
  umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle = {
  0,
  &umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_members,
  get_message_typesupport_handle_function,
  &umi_rtx_interfaces__msg__GameData__get_type_hash,
  &umi_rtx_interfaces__msg__GameData__get_type_description,
  &umi_rtx_interfaces__msg__GameData__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_umi_rtx_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, umi_rtx_interfaces, msg, GameData)() {
  umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, umi_rtx_interfaces, msg, Board)();
  if (!umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle.typesupport_identifier) {
    umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &umi_rtx_interfaces__msg__GameData__rosidl_typesupport_introspection_c__GameData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
