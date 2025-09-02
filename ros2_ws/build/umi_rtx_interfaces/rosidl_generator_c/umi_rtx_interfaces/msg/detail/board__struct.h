// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__STRUCT_H_
#define UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Board in the package umi_rtx_interfaces.
typedef struct umi_rtx_interfaces__msg__Board
{
  int32_t data[9];
} umi_rtx_interfaces__msg__Board;

// Struct for a sequence of umi_rtx_interfaces__msg__Board.
typedef struct umi_rtx_interfaces__msg__Board__Sequence
{
  umi_rtx_interfaces__msg__Board * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} umi_rtx_interfaces__msg__Board__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__STRUCT_H_
