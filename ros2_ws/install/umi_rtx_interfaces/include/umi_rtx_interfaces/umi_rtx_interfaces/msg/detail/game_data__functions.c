// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice
#include "umi_rtx_interfaces/msg/detail/game_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `board`
#include "umi_rtx_interfaces/msg/detail/board__functions.h"
// Member `moveshistory`
// Member `primarymsg`
// Member `secondarymsg`
#include "rosidl_runtime_c/string_functions.h"

bool
umi_rtx_interfaces__msg__GameData__init(umi_rtx_interfaces__msg__GameData * msg)
{
  if (!msg) {
    return false;
  }
  // board
  if (!umi_rtx_interfaces__msg__Board__init(&msg->board)) {
    umi_rtx_interfaces__msg__GameData__fini(msg);
    return false;
  }
  // moveshistory
  for (size_t i = 0; i < 9; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->moveshistory[i])) {
      umi_rtx_interfaces__msg__GameData__fini(msg);
      return false;
    }
  }
  // primarymsg
  if (!rosidl_runtime_c__String__init(&msg->primarymsg)) {
    umi_rtx_interfaces__msg__GameData__fini(msg);
    return false;
  }
  // secondarymsg
  if (!rosidl_runtime_c__String__init(&msg->secondarymsg)) {
    umi_rtx_interfaces__msg__GameData__fini(msg);
    return false;
  }
  // isrobotturn
  // isgamestarted
  return true;
}

void
umi_rtx_interfaces__msg__GameData__fini(umi_rtx_interfaces__msg__GameData * msg)
{
  if (!msg) {
    return;
  }
  // board
  umi_rtx_interfaces__msg__Board__fini(&msg->board);
  // moveshistory
  for (size_t i = 0; i < 9; ++i) {
    rosidl_runtime_c__String__fini(&msg->moveshistory[i]);
  }
  // primarymsg
  rosidl_runtime_c__String__fini(&msg->primarymsg);
  // secondarymsg
  rosidl_runtime_c__String__fini(&msg->secondarymsg);
  // isrobotturn
  // isgamestarted
}

bool
umi_rtx_interfaces__msg__GameData__are_equal(const umi_rtx_interfaces__msg__GameData * lhs, const umi_rtx_interfaces__msg__GameData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // board
  if (!umi_rtx_interfaces__msg__Board__are_equal(
      &(lhs->board), &(rhs->board)))
  {
    return false;
  }
  // moveshistory
  for (size_t i = 0; i < 9; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->moveshistory[i]), &(rhs->moveshistory[i])))
    {
      return false;
    }
  }
  // primarymsg
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->primarymsg), &(rhs->primarymsg)))
  {
    return false;
  }
  // secondarymsg
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->secondarymsg), &(rhs->secondarymsg)))
  {
    return false;
  }
  // isrobotturn
  if (lhs->isrobotturn != rhs->isrobotturn) {
    return false;
  }
  // isgamestarted
  if (lhs->isgamestarted != rhs->isgamestarted) {
    return false;
  }
  return true;
}

bool
umi_rtx_interfaces__msg__GameData__copy(
  const umi_rtx_interfaces__msg__GameData * input,
  umi_rtx_interfaces__msg__GameData * output)
{
  if (!input || !output) {
    return false;
  }
  // board
  if (!umi_rtx_interfaces__msg__Board__copy(
      &(input->board), &(output->board)))
  {
    return false;
  }
  // moveshistory
  for (size_t i = 0; i < 9; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->moveshistory[i]), &(output->moveshistory[i])))
    {
      return false;
    }
  }
  // primarymsg
  if (!rosidl_runtime_c__String__copy(
      &(input->primarymsg), &(output->primarymsg)))
  {
    return false;
  }
  // secondarymsg
  if (!rosidl_runtime_c__String__copy(
      &(input->secondarymsg), &(output->secondarymsg)))
  {
    return false;
  }
  // isrobotturn
  output->isrobotturn = input->isrobotturn;
  // isgamestarted
  output->isgamestarted = input->isgamestarted;
  return true;
}

umi_rtx_interfaces__msg__GameData *
umi_rtx_interfaces__msg__GameData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  umi_rtx_interfaces__msg__GameData * msg = (umi_rtx_interfaces__msg__GameData *)allocator.allocate(sizeof(umi_rtx_interfaces__msg__GameData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(umi_rtx_interfaces__msg__GameData));
  bool success = umi_rtx_interfaces__msg__GameData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
umi_rtx_interfaces__msg__GameData__destroy(umi_rtx_interfaces__msg__GameData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    umi_rtx_interfaces__msg__GameData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
umi_rtx_interfaces__msg__GameData__Sequence__init(umi_rtx_interfaces__msg__GameData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  umi_rtx_interfaces__msg__GameData * data = NULL;

  if (size) {
    data = (umi_rtx_interfaces__msg__GameData *)allocator.zero_allocate(size, sizeof(umi_rtx_interfaces__msg__GameData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = umi_rtx_interfaces__msg__GameData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        umi_rtx_interfaces__msg__GameData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
umi_rtx_interfaces__msg__GameData__Sequence__fini(umi_rtx_interfaces__msg__GameData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      umi_rtx_interfaces__msg__GameData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

umi_rtx_interfaces__msg__GameData__Sequence *
umi_rtx_interfaces__msg__GameData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  umi_rtx_interfaces__msg__GameData__Sequence * array = (umi_rtx_interfaces__msg__GameData__Sequence *)allocator.allocate(sizeof(umi_rtx_interfaces__msg__GameData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = umi_rtx_interfaces__msg__GameData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
umi_rtx_interfaces__msg__GameData__Sequence__destroy(umi_rtx_interfaces__msg__GameData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    umi_rtx_interfaces__msg__GameData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
umi_rtx_interfaces__msg__GameData__Sequence__are_equal(const umi_rtx_interfaces__msg__GameData__Sequence * lhs, const umi_rtx_interfaces__msg__GameData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!umi_rtx_interfaces__msg__GameData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
umi_rtx_interfaces__msg__GameData__Sequence__copy(
  const umi_rtx_interfaces__msg__GameData__Sequence * input,
  umi_rtx_interfaces__msg__GameData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(umi_rtx_interfaces__msg__GameData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    umi_rtx_interfaces__msg__GameData * data =
      (umi_rtx_interfaces__msg__GameData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!umi_rtx_interfaces__msg__GameData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          umi_rtx_interfaces__msg__GameData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!umi_rtx_interfaces__msg__GameData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
