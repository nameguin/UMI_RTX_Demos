// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from umi_rtx_interfaces:msg/GameData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "umi_rtx_interfaces/msg/detail/game_data__struct.h"
#include "umi_rtx_interfaces/msg/detail/game_data__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

bool umi_rtx_interfaces__msg__board__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * umi_rtx_interfaces__msg__board__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool umi_rtx_interfaces__msg__game_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("umi_rtx_interfaces.msg._game_data.GameData", full_classname_dest, 42) == 0);
  }
  umi_rtx_interfaces__msg__GameData * ros_message = _ros_message;
  {  // board
    PyObject * field = PyObject_GetAttrString(_pymsg, "board");
    if (!field) {
      return false;
    }
    if (!umi_rtx_interfaces__msg__board__convert_from_py(field, &ros_message->board)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // moveshistory
    PyObject * field = PyObject_GetAttrString(_pymsg, "moveshistory");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'moveshistory'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = 9;
      rosidl_runtime_c__String * dest = ros_message->moveshistory;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyUnicode_Check(item));
        PyObject * encoded_item = PyUnicode_AsUTF8String(item);
        if (!encoded_item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        rosidl_runtime_c__String__assign(&dest[i], PyBytes_AS_STRING(encoded_item));
        Py_DECREF(encoded_item);
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // primarymsg
    PyObject * field = PyObject_GetAttrString(_pymsg, "primarymsg");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->primarymsg, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // secondarymsg
    PyObject * field = PyObject_GetAttrString(_pymsg, "secondarymsg");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->secondarymsg, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // isrobotturn
    PyObject * field = PyObject_GetAttrString(_pymsg, "isrobotturn");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->isrobotturn = (Py_True == field);
    Py_DECREF(field);
  }
  {  // isgamestarted
    PyObject * field = PyObject_GetAttrString(_pymsg, "isgamestarted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->isgamestarted = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * umi_rtx_interfaces__msg__game_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GameData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("umi_rtx_interfaces.msg._game_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GameData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  umi_rtx_interfaces__msg__GameData * ros_message = (umi_rtx_interfaces__msg__GameData *)raw_ros_message;
  {  // board
    PyObject * field = NULL;
    field = umi_rtx_interfaces__msg__board__convert_to_py(&ros_message->board);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "board", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // moveshistory
    PyObject * field = NULL;
    size_t size = 9;
    rosidl_runtime_c__String * src = ros_message->moveshistory;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      PyObject * decoded_item = PyUnicode_DecodeUTF8(src[i].data, strlen(src[i].data), "replace");
      if (!decoded_item) {
        return NULL;
      }
      int rc = PyList_SetItem(field, i, decoded_item);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "moveshistory", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // primarymsg
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->primarymsg.data,
      strlen(ros_message->primarymsg.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "primarymsg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // secondarymsg
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->secondarymsg.data,
      strlen(ros_message->secondarymsg.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "secondarymsg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // isrobotturn
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->isrobotturn ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "isrobotturn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // isgamestarted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->isgamestarted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "isgamestarted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
