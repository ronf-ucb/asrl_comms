"""autogenerated by genpy from asrl_sensor_msgs/GgaInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import std_msgs.msg

class GgaInfo(genpy.Message):
  _md5sum = "52c711feed09f0cce626b8ded662359f"
  _type = "asrl_sensor_msgs/GgaInfo"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# Utc time
time utcTime

# Lattitude
float32 latitude
string latCoordinate

# Longitude
float32 longitude
string longCoordinate

# Geoidal Altitude (m)
float32 geoidAlt

# Rtk code: 1 is Stand Alone, 4 if fixed RTK, 5 is float RTK, invalid otherwise
uint8 rtkCode

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','utcTime','latitude','latCoordinate','longitude','longCoordinate','geoidAlt','rtkCode']
  _slot_types = ['std_msgs/Header','time','float32','string','float32','string','float32','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,utcTime,latitude,latCoordinate,longitude,longCoordinate,geoidAlt,rtkCode

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GgaInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.utcTime is None:
        self.utcTime = genpy.Time()
      if self.latitude is None:
        self.latitude = 0.
      if self.latCoordinate is None:
        self.latCoordinate = ''
      if self.longitude is None:
        self.longitude = 0.
      if self.longCoordinate is None:
        self.longCoordinate = ''
      if self.geoidAlt is None:
        self.geoidAlt = 0.
      if self.rtkCode is None:
        self.rtkCode = 0
    else:
      self.header = std_msgs.msg.Header()
      self.utcTime = genpy.Time()
      self.latitude = 0.
      self.latCoordinate = ''
      self.longitude = 0.
      self.longCoordinate = ''
      self.geoidAlt = 0.
      self.rtkCode = 0

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
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2If.pack(_x.utcTime.secs, _x.utcTime.nsecs, _x.latitude))
      _x = self.latCoordinate
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_f.pack(self.longitude))
      _x = self.longCoordinate
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_fB.pack(_x.geoidAlt, _x.rtkCode))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.utcTime is None:
        self.utcTime = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.utcTime.secs, _x.utcTime.nsecs, _x.latitude,) = _struct_2If.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.latCoordinate = str[start:end].decode('utf-8')
      else:
        self.latCoordinate = str[start:end]
      start = end
      end += 4
      (self.longitude,) = _struct_f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.longCoordinate = str[start:end].decode('utf-8')
      else:
        self.longCoordinate = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.geoidAlt, _x.rtkCode,) = _struct_fB.unpack(str[start:end])
      self.utcTime.canon()
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
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2If.pack(_x.utcTime.secs, _x.utcTime.nsecs, _x.latitude))
      _x = self.latCoordinate
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_f.pack(self.longitude))
      _x = self.longCoordinate
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_fB.pack(_x.geoidAlt, _x.rtkCode))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.utcTime is None:
        self.utcTime = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.utcTime.secs, _x.utcTime.nsecs, _x.latitude,) = _struct_2If.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.latCoordinate = str[start:end].decode('utf-8')
      else:
        self.latCoordinate = str[start:end]
      start = end
      end += 4
      (self.longitude,) = _struct_f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.longCoordinate = str[start:end].decode('utf-8')
      else:
        self.longCoordinate = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.geoidAlt, _x.rtkCode,) = _struct_fB.unpack(str[start:end])
      self.utcTime.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2If = struct.Struct("<2If")
_struct_3I = struct.Struct("<3I")
_struct_fB = struct.Struct("<fB")
_struct_f = struct.Struct("<f")