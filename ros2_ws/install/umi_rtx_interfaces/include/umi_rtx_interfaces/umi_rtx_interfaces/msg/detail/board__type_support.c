// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "umi_rtx_interfaces/msg/detail/board__rosidl_typesupport_introspection_c.h"
#include "umi_rtx_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "umi_rtx_interfaces/msg/detail/board__functions.h"
#include "umi_rtx_interfaces/msg/detail/board__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  umi_rtx_interfaces__msg__Board__init(message_memory);
}

void umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_fini_function(void * message_memory)
{
  umi_rtx_interfaces__msg__Board__fini(message_memory);
}

size_t umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__size_function__Board__data(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__get_const_function__Board__data(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__get_function__Board__data(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__fetch_function__Board__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__get_const_function__Board__data(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__assign_function__Board__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__get_function__Board__data(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(umi_rtx_interfaces__msg__Board, data),  // bytes offset in struct
    NULL,  // default value
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__size_function__Board__data,  // size() function pointer
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__get_const_function__Board__data,  // get_const(index) function pointer
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__get_function__Board__data,  // get(index) function pointer
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__fetch_function__Board__data,  // fetch(index, &value) function pointer
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__assign_function__Board__data,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_members = {
  "umi_rtx_interfaces__msg",  // message namespace
  "Board",  // message name
  1,  // number of fields
  sizeof(umi_rtx_interfaces__msg__Board),
  umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_member_array,  // message members
  umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_init_function,  // function to initialize message memory (memory has to be allocated)
  umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_type_support_handle = {
  0,
  &umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_members,
  get_message_typesupport_handle_function,
  &umi_rtx_interfaces__msg__Board__get_type_hash,
  &umi_rtx_interfaces__msg__Board__get_type_description,
  &umi_rtx_interfaces__msg__Board__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_umi_rtx_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, umi_rtx_interfaces, msg, Board)() {
  if (!umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_type_support_handle.typesupport_identifier) {
    umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &umi_rtx_interfaces__msg__Board__rosidl_typesupport_introspection_c__Board_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
