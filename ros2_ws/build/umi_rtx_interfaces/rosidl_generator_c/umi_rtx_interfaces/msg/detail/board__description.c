// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#include "umi_rtx_interfaces/msg/detail/board__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
const rosidl_type_hash_t *
umi_rtx_interfaces__msg__Board__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xda, 0x88, 0xa0, 0xef, 0x4c, 0x82, 0x89, 0x13,
      0x2b, 0xf5, 0xad, 0x4c, 0x0b, 0x54, 0x02, 0x1e,
      0xe1, 0x2e, 0xd2, 0x65, 0x19, 0xd8, 0x7a, 0xe5,
      0xd8, 0x0a, 0x6a, 0x56, 0x17, 0x5e, 0xd7, 0xed,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char umi_rtx_interfaces__msg__Board__TYPE_NAME[] = "umi_rtx_interfaces/msg/Board";

// Define type names, field names, and default values
static char umi_rtx_interfaces__msg__Board__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field umi_rtx_interfaces__msg__Board__FIELDS[] = {
  {
    {umi_rtx_interfaces__msg__Board__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
umi_rtx_interfaces__msg__Board__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {umi_rtx_interfaces__msg__Board__TYPE_NAME, 28, 28},
      {umi_rtx_interfaces__msg__Board__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32[9] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
umi_rtx_interfaces__msg__Board__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {umi_rtx_interfaces__msg__Board__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 14, 14},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
umi_rtx_interfaces__msg__Board__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *umi_rtx_interfaces__msg__Board__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
