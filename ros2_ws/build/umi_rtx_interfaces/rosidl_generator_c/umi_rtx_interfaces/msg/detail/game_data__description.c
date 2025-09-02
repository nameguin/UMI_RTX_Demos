// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice

#include "umi_rtx_interfaces/msg/detail/game_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
const rosidl_type_hash_t *
umi_rtx_interfaces__msg__GameData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x42, 0x69, 0x0e, 0x73, 0xcb, 0x61, 0xc7, 0x07,
      0xd0, 0x50, 0xbe, 0x51, 0x6c, 0xd1, 0xf8, 0x5c,
      0x59, 0x1a, 0x3c, 0xa6, 0x86, 0xd5, 0x51, 0x95,
      0xc3, 0xfd, 0x0a, 0x53, 0xe9, 0x1f, 0x0c, 0x97,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "umi_rtx_interfaces/msg/detail/board__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t umi_rtx_interfaces__msg__Board__EXPECTED_HASH = {1, {
    0xda, 0x88, 0xa0, 0xef, 0x4c, 0x82, 0x89, 0x13,
    0x2b, 0xf5, 0xad, 0x4c, 0x0b, 0x54, 0x02, 0x1e,
    0xe1, 0x2e, 0xd2, 0x65, 0x19, 0xd8, 0x7a, 0xe5,
    0xd8, 0x0a, 0x6a, 0x56, 0x17, 0x5e, 0xd7, 0xed,
  }};
#endif

static char umi_rtx_interfaces__msg__GameData__TYPE_NAME[] = "umi_rtx_interfaces/msg/GameData";
static char umi_rtx_interfaces__msg__Board__TYPE_NAME[] = "umi_rtx_interfaces/msg/Board";

// Define type names, field names, and default values
static char umi_rtx_interfaces__msg__GameData__FIELD_NAME__board[] = "board";
static char umi_rtx_interfaces__msg__GameData__FIELD_NAME__moveshistory[] = "moveshistory";
static char umi_rtx_interfaces__msg__GameData__FIELD_NAME__primarymsg[] = "primarymsg";
static char umi_rtx_interfaces__msg__GameData__FIELD_NAME__secondarymsg[] = "secondarymsg";
static char umi_rtx_interfaces__msg__GameData__FIELD_NAME__isrobotturn[] = "isrobotturn";
static char umi_rtx_interfaces__msg__GameData__FIELD_NAME__isgamestarted[] = "isgamestarted";

static rosidl_runtime_c__type_description__Field umi_rtx_interfaces__msg__GameData__FIELDS[] = {
  {
    {umi_rtx_interfaces__msg__GameData__FIELD_NAME__board, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {umi_rtx_interfaces__msg__Board__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {umi_rtx_interfaces__msg__GameData__FIELD_NAME__moveshistory, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {umi_rtx_interfaces__msg__GameData__FIELD_NAME__primarymsg, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {umi_rtx_interfaces__msg__GameData__FIELD_NAME__secondarymsg, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {umi_rtx_interfaces__msg__GameData__FIELD_NAME__isrobotturn, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {umi_rtx_interfaces__msg__GameData__FIELD_NAME__isgamestarted, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription umi_rtx_interfaces__msg__GameData__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {umi_rtx_interfaces__msg__Board__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
umi_rtx_interfaces__msg__GameData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {umi_rtx_interfaces__msg__GameData__TYPE_NAME, 31, 31},
      {umi_rtx_interfaces__msg__GameData__FIELDS, 6, 6},
    },
    {umi_rtx_interfaces__msg__GameData__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&umi_rtx_interfaces__msg__Board__EXPECTED_HASH, umi_rtx_interfaces__msg__Board__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = umi_rtx_interfaces__msg__Board__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "Board board\n"
  "string[9] moveshistory\n"
  "string primarymsg\n"
  "string secondarymsg\n"
  "bool isrobotturn\n"
  "bool isgamestarted";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
umi_rtx_interfaces__msg__GameData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {umi_rtx_interfaces__msg__GameData__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 108, 108},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
umi_rtx_interfaces__msg__GameData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *umi_rtx_interfaces__msg__GameData__get_individual_type_description_source(NULL),
    sources[1] = *umi_rtx_interfaces__msg__Board__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
