// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from umi_rtx_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__FUNCTIONS_H_
#define UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "umi_rtx_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "umi_rtx_interfaces/msg/detail/board__struct.h"

/// Initialize msg/Board message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * umi_rtx_interfaces__msg__Board
 * )) before or use
 * umi_rtx_interfaces__msg__Board__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
bool
umi_rtx_interfaces__msg__Board__init(umi_rtx_interfaces__msg__Board * msg);

/// Finalize msg/Board message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
void
umi_rtx_interfaces__msg__Board__fini(umi_rtx_interfaces__msg__Board * msg);

/// Create msg/Board message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * umi_rtx_interfaces__msg__Board__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
umi_rtx_interfaces__msg__Board *
umi_rtx_interfaces__msg__Board__create();

/// Destroy msg/Board message.
/**
 * It calls
 * umi_rtx_interfaces__msg__Board__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
void
umi_rtx_interfaces__msg__Board__destroy(umi_rtx_interfaces__msg__Board * msg);

/// Check for msg/Board message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
bool
umi_rtx_interfaces__msg__Board__are_equal(const umi_rtx_interfaces__msg__Board * lhs, const umi_rtx_interfaces__msg__Board * rhs);

/// Copy a msg/Board message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
bool
umi_rtx_interfaces__msg__Board__copy(
  const umi_rtx_interfaces__msg__Board * input,
  umi_rtx_interfaces__msg__Board * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
const rosidl_type_hash_t *
umi_rtx_interfaces__msg__Board__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
umi_rtx_interfaces__msg__Board__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
const rosidl_runtime_c__type_description__TypeSource *
umi_rtx_interfaces__msg__Board__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
umi_rtx_interfaces__msg__Board__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/Board messages.
/**
 * It allocates the memory for the number of elements and calls
 * umi_rtx_interfaces__msg__Board__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
bool
umi_rtx_interfaces__msg__Board__Sequence__init(umi_rtx_interfaces__msg__Board__Sequence * array, size_t size);

/// Finalize array of msg/Board messages.
/**
 * It calls
 * umi_rtx_interfaces__msg__Board__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
void
umi_rtx_interfaces__msg__Board__Sequence__fini(umi_rtx_interfaces__msg__Board__Sequence * array);

/// Create array of msg/Board messages.
/**
 * It allocates the memory for the array and calls
 * umi_rtx_interfaces__msg__Board__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
umi_rtx_interfaces__msg__Board__Sequence *
umi_rtx_interfaces__msg__Board__Sequence__create(size_t size);

/// Destroy array of msg/Board messages.
/**
 * It calls
 * umi_rtx_interfaces__msg__Board__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
void
umi_rtx_interfaces__msg__Board__Sequence__destroy(umi_rtx_interfaces__msg__Board__Sequence * array);

/// Check for msg/Board message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
bool
umi_rtx_interfaces__msg__Board__Sequence__are_equal(const umi_rtx_interfaces__msg__Board__Sequence * lhs, const umi_rtx_interfaces__msg__Board__Sequence * rhs);

/// Copy an array of msg/Board messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_umi_rtx_interfaces
bool
umi_rtx_interfaces__msg__Board__Sequence__copy(
  const umi_rtx_interfaces__msg__Board__Sequence * input,
  umi_rtx_interfaces__msg__Board__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UMI_RTX_INTERFACES__MSG__DETAIL__BOARD__FUNCTIONS_H_
