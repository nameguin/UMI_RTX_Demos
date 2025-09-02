// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice
#include "umi_rtx_interfaces/msg/detail/game_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "umi_rtx_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "umi_rtx_interfaces/msg/detail/game_data__struct.h"
#include "umi_rtx_interfaces/msg/detail/game_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // moveshistory, primarymsg, secondarymsg
#include "rosidl_runtime_c/string_functions.h"  // moveshistory, primarymsg, secondarymsg
#include "umi_rtx_interfaces/msg/detail/board__functions.h"  // board

// forward declare type support functions
size_t get_serialized_size_umi_rtx_interfaces__msg__Board(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_umi_rtx_interfaces__msg__Board(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, umi_rtx_interfaces, msg, Board)();


using _GameData__ros_msg_type = umi_rtx_interfaces__msg__GameData;

static bool _GameData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GameData__ros_msg_type * ros_message = static_cast<const _GameData__ros_msg_type *>(untyped_ros_message);
  // Field name: board
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, umi_rtx_interfaces, msg, Board
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->board, cdr))
    {
      return false;
    }
  }

  // Field name: moveshistory
  {
    size_t size = 9;
    auto array_ptr = ros_message->moveshistory;
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: primarymsg
  {
    const rosidl_runtime_c__String * str = &ros_message->primarymsg;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: secondarymsg
  {
    const rosidl_runtime_c__String * str = &ros_message->secondarymsg;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: isrobotturn
  {
    cdr << (ros_message->isrobotturn ? true : false);
  }

  // Field name: isgamestarted
  {
    cdr << (ros_message->isgamestarted ? true : false);
  }

  return true;
}

static bool _GameData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GameData__ros_msg_type * ros_message = static_cast<_GameData__ros_msg_type *>(untyped_ros_message);
  // Field name: board
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, umi_rtx_interfaces, msg, Board
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->board))
    {
      return false;
    }
  }

  // Field name: moveshistory
  {
    size_t size = 9;
    auto array_ptr = ros_message->moveshistory;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'moveshistory'\n");
        return false;
      }
    }
  }

  // Field name: primarymsg
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->primarymsg.data) {
      rosidl_runtime_c__String__init(&ros_message->primarymsg);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->primarymsg,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'primarymsg'\n");
      return false;
    }
  }

  // Field name: secondarymsg
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->secondarymsg.data) {
      rosidl_runtime_c__String__init(&ros_message->secondarymsg);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->secondarymsg,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'secondarymsg'\n");
      return false;
    }
  }

  // Field name: isrobotturn
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->isrobotturn = tmp ? true : false;
  }

  // Field name: isgamestarted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->isgamestarted = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_umi_rtx_interfaces
size_t get_serialized_size_umi_rtx_interfaces__msg__GameData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GameData__ros_msg_type * ros_message = static_cast<const _GameData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name board

  current_alignment += get_serialized_size_umi_rtx_interfaces__msg__Board(
    &(ros_message->board), current_alignment);
  // field.name moveshistory
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->moveshistory;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name primarymsg
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->primarymsg.size + 1);
  // field.name secondarymsg
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->secondarymsg.size + 1);
  // field.name isrobotturn
  {
    size_t item_size = sizeof(ros_message->isrobotturn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name isgamestarted
  {
    size_t item_size = sizeof(ros_message->isgamestarted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GameData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_umi_rtx_interfaces__msg__GameData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_umi_rtx_interfaces
size_t max_serialized_size_umi_rtx_interfaces__msg__GameData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: board
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_umi_rtx_interfaces__msg__Board(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: moveshistory
  {
    size_t array_size = 9;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: primarymsg
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: secondarymsg
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: isrobotturn
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: isgamestarted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = umi_rtx_interfaces__msg__GameData;
    is_plain =
      (
      offsetof(DataType, isgamestarted) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GameData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_umi_rtx_interfaces__msg__GameData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GameData = {
  "umi_rtx_interfaces::msg",
  "GameData",
  _GameData__cdr_serialize,
  _GameData__cdr_deserialize,
  _GameData__get_serialized_size,
  _GameData__max_serialized_size
};

static rosidl_message_type_support_t _GameData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GameData,
  get_message_typesupport_handle_function,
  &umi_rtx_interfaces__msg__GameData__get_type_hash,
  &umi_rtx_interfaces__msg__GameData__get_type_description,
  &umi_rtx_interfaces__msg__GameData__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, umi_rtx_interfaces, msg, GameData)() {
  return &_GameData__type_support;
}

#if defined(__cplusplus)
}
#endif
