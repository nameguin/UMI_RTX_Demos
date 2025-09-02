// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice
#include "umi_rtx_interfaces/msg/detail/board__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
umi_rtx_interfaces__msg__Board__init(umi_rtx_interfaces__msg__Board * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
umi_rtx_interfaces__msg__Board__fini(umi_rtx_interfaces__msg__Board * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
umi_rtx_interfaces__msg__Board__are_equal(const umi_rtx_interfaces__msg__Board * lhs, const umi_rtx_interfaces__msg__Board * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->data[i] != rhs->data[i]) {
      return false;
    }
  }
  return true;
}

bool
umi_rtx_interfaces__msg__Board__copy(
  const umi_rtx_interfaces__msg__Board * input,
  umi_rtx_interfaces__msg__Board * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  for (size_t i = 0; i < 9; ++i) {
    output->data[i] = input->data[i];
  }
  return true;
}

umi_rtx_interfaces__msg__Board *
umi_rtx_interfaces__msg__Board__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  umi_rtx_interfaces__msg__Board * msg = (umi_rtx_interfaces__msg__Board *)allocator.allocate(sizeof(umi_rtx_interfaces__msg__Board), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(umi_rtx_interfaces__msg__Board));
  bool success = umi_rtx_interfaces__msg__Board__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
umi_rtx_interfaces__msg__Board__destroy(umi_rtx_interfaces__msg__Board * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    umi_rtx_interfaces__msg__Board__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
umi_rtx_interfaces__msg__Board__Sequence__init(umi_rtx_interfaces__msg__Board__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  umi_rtx_interfaces__msg__Board * data = NULL;

  if (size) {
    data = (umi_rtx_interfaces__msg__Board *)allocator.zero_allocate(size, sizeof(umi_rtx_interfaces__msg__Board), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = umi_rtx_interfaces__msg__Board__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        umi_rtx_interfaces__msg__Board__fini(&data[i - 1]);
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
umi_rtx_interfaces__msg__Board__Sequence__fini(umi_rtx_interfaces__msg__Board__Sequence * array)
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
      umi_rtx_interfaces__msg__Board__fini(&array->data[i]);
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

umi_rtx_interfaces__msg__Board__Sequence *
umi_rtx_interfaces__msg__Board__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  umi_rtx_interfaces__msg__Board__Sequence * array = (umi_rtx_interfaces__msg__Board__Sequence *)allocator.allocate(sizeof(umi_rtx_interfaces__msg__Board__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = umi_rtx_interfaces__msg__Board__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
umi_rtx_interfaces__msg__Board__Sequence__destroy(umi_rtx_interfaces__msg__Board__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    umi_rtx_interfaces__msg__Board__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
umi_rtx_interfaces__msg__Board__Sequence__are_equal(const umi_rtx_interfaces__msg__Board__Sequence * lhs, const umi_rtx_interfaces__msg__Board__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!umi_rtx_interfaces__msg__Board__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
umi_rtx_interfaces__msg__Board__Sequence__copy(
  const umi_rtx_interfaces__msg__Board__Sequence * input,
  umi_rtx_interfaces__msg__Board__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(umi_rtx_interfaces__msg__Board);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    umi_rtx_interfaces__msg__Board * data =
      (umi_rtx_interfaces__msg__Board *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!umi_rtx_interfaces__msg__Board__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          umi_rtx_interfaces__msg__Board__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!umi_rtx_interfaces__msg__Board__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
