# generated from rosidl_generator_py/resource/_idl.py.em
# with input from umi_rtx_interfaces:msg/GameData.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GameData(type):
    """Metaclass of message 'GameData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('umi_rtx_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'umi_rtx_interfaces.msg.GameData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__game_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__game_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__game_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__game_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__game_data

            from umi_rtx_interfaces.msg import Board
            if Board.__class__._TYPE_SUPPORT is None:
                Board.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GameData(metaclass=Metaclass_GameData):
    """Message class 'GameData'."""

    __slots__ = [
        '_board',
        '_moveshistory',
        '_primarymsg',
        '_secondarymsg',
        '_isrobotturn',
        '_isgamestarted',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'board': 'umi_rtx_interfaces/Board',
        'moveshistory': 'string[9]',
        'primarymsg': 'string',
        'secondarymsg': 'string',
        'isrobotturn': 'boolean',
        'isgamestarted': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['umi_rtx_interfaces', 'msg'], 'Board'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.UnboundedString(), 9),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from umi_rtx_interfaces.msg import Board
        self.board = kwargs.get('board', Board())
        self.moveshistory = kwargs.get(
            'moveshistory',
            [str() for x in range(9)]
        )
        self.primarymsg = kwargs.get('primarymsg', str())
        self.secondarymsg = kwargs.get('secondarymsg', str())
        self.isrobotturn = kwargs.get('isrobotturn', bool())
        self.isgamestarted = kwargs.get('isgamestarted', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.board != other.board:
            return False
        if self.moveshistory != other.moveshistory:
            return False
        if self.primarymsg != other.primarymsg:
            return False
        if self.secondarymsg != other.secondarymsg:
            return False
        if self.isrobotturn != other.isrobotturn:
            return False
        if self.isgamestarted != other.isgamestarted:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def board(self):
        """Message field 'board'."""
        return self._board

    @board.setter
    def board(self, value):
        if self._check_fields:
            from umi_rtx_interfaces.msg import Board
            assert \
                isinstance(value, Board), \
                "The 'board' field must be a sub message of type 'Board'"
        self._board = value

    @builtins.property
    def moveshistory(self):
        """Message field 'moveshistory'."""
        return self._moveshistory

    @moveshistory.setter
    def moveshistory(self, value):
        if self._check_fields:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'moveshistory' field must be a set or sequence with length 9 and each value of type 'str'"
        self._moveshistory = value

    @builtins.property
    def primarymsg(self):
        """Message field 'primarymsg'."""
        return self._primarymsg

    @primarymsg.setter
    def primarymsg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'primarymsg' field must be of type 'str'"
        self._primarymsg = value

    @builtins.property
    def secondarymsg(self):
        """Message field 'secondarymsg'."""
        return self._secondarymsg

    @secondarymsg.setter
    def secondarymsg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'secondarymsg' field must be of type 'str'"
        self._secondarymsg = value

    @builtins.property
    def isrobotturn(self):
        """Message field 'isrobotturn'."""
        return self._isrobotturn

    @isrobotturn.setter
    def isrobotturn(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'isrobotturn' field must be of type 'bool'"
        self._isrobotturn = value

    @builtins.property
    def isgamestarted(self):
        """Message field 'isgamestarted'."""
        return self._isgamestarted

    @isgamestarted.setter
    def isgamestarted(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'isgamestarted' field must be of type 'bool'"
        self._isgamestarted = value
