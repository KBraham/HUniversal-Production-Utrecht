"""autogenerated by genpy from crateLocatorNode/getCrateRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class getCrateRequest(genpy.Message):
  _md5sum = "c1f3d28f1b044c871e6eff2e9fc3c667"
  _type = "crateLocatorNode/getCrateRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string name

"""
  __slots__ = ['name']
  _slot_types = ['string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(getCrateRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
    else:
      self.name = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
"""autogenerated by genpy from crateLocatorNode/getCrateResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import crateLocatorNode.msg

class getCrateResponse(genpy.Message):
  _md5sum = "9f25c216b34381eff127339913361646"
  _type = "crateLocatorNode/getCrateResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 state
CrateMsg crate


================================================================================
MSG: crateLocatorNode/CrateMsg
string name
float32 x
float32 y
float32 angle


"""
  __slots__ = ['state','crate']
  _slot_types = ['int32','crateLocatorNode/CrateMsg']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       state,crate

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(getCrateResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.state is None:
        self.state = 0
      if self.crate is None:
        self.crate = crateLocatorNode.msg.CrateMsg()
    else:
      self.state = 0
      self.crate = crateLocatorNode.msg.CrateMsg()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_i.pack(self.state))
      _x = self.crate.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3f.pack(_x.crate.x, _x.crate.y, _x.crate.angle))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.crate is None:
        self.crate = crateLocatorNode.msg.CrateMsg()
      end = 0
      start = end
      end += 4
      (self.state,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.crate.name = str[start:end].decode('utf-8')
      else:
        self.crate.name = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.crate.x, _x.crate.y, _x.crate.angle,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_struct_i.pack(self.state))
      _x = self.crate.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3f.pack(_x.crate.x, _x.crate.y, _x.crate.angle))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.crate is None:
        self.crate = crateLocatorNode.msg.CrateMsg()
      end = 0
      start = end
      end += 4
      (self.state,) = _struct_i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.crate.name = str[start:end].decode('utf-8')
      else:
        self.crate.name = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.crate.x, _x.crate.y, _x.crate.angle,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
_struct_3f = struct.Struct("<3f")
class getCrate(object):
  _type          = 'crateLocatorNode/getCrate'
  _md5sum = 'b4251b3d3527c8380a2ba227b5b5ea13'
  _request_class  = getCrateRequest
  _response_class = getCrateResponse