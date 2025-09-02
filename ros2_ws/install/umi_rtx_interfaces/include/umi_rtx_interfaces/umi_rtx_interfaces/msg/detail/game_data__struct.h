// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__STRUCT_H_
#define UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'board'
#include "umi_rtx_interfaces/msg/detail/board__struct.h"
// Member 'moveshistory'
// Member 'primarymsg'
// Member 'secondarymsg'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GameData in the package umi_rtx_interfaces.
typedef struct umi_rtx_interfaces__msg__GameData
{
  umi_rtx_interfaces__msg__Board board;
  rosidl_runtime_c__String moveshistory[9];
  rosidl_runtime_c__String primarymsg;
  rosidl_runtime_c__String secondarymsg;
  bool isrobotturn;
  bool isgamestarted;
} umi_rtx_interfaces__msg__GameData;

// Struct for a sequence of umi_rtx_interfaces__msg__GameData.
typedef struct umi_rtx_interfaces__msg__GameData__Sequence
{
  umi_rtx_interfaces__msg__GameData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} umi_rtx_interfaces__msg__GameData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__GAME_DATA__STRUCT_H_
