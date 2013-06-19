"""autogenerated by genpy from asrl_sensor_msgs/SunSensorData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class SunSensorData(genpy.Message):
  _md5sum = "06161ef49dc9ea3e276d16646600287d"
  _type = "asrl_sensor_msgs/SunSensorData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# Inclinometer data
uint16[] image
float32 tempCelcius

# Sun Vector 
float32[] sunVector

# Error code 1 from the sun vector computation algorithm
# The value of the error code should be interpreted as follows:
# >= 0: Successful image.
#   -1: Error returns during NLSQ fit, solution may be valid to nearest pixel.
#   -2: Too many peaks, solution not assigned.
#   -3: Too few peaks, solution not assigned 
#   -4: Appropriate number of peaks found, but fit is of poor quality.
int32 errorCode1

# Error code 2 from the sun vector computation algorithm
# The value of the error code should be interpreted as follows:
#    0: Good geometry.
#   -5: Imaginary solution.
int32 errorCode2
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
  __slots__ = ['header','image','tempCelcius','sunVector','errorCode1','errorCode2']
  _slot_types = ['std_msgs/Header','uint16[]','float32','float32[]','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,image,tempCelcius,sunVector,errorCode1,errorCode2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SunSensorData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.image is None:
        self.image = []
      if self.tempCelcius is None:
        self.tempCelcius = 0.
      if self.sunVector is None:
        self.sunVector = []
      if self.errorCode1 is None:
        self.errorCode1 = 0
      if self.errorCode2 is None:
        self.errorCode2 = 0
    else:
      self.header = std_msgs.msg.Header()
      self.image = []
      self.tempCelcius = 0.
      self.sunVector = []
      self.errorCode1 = 0
      self.errorCode2 = 0

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
      length = len(self.image)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(struct.pack(pattern, *self.image))
      buff.write(_struct_f.pack(self.tempCelcius))
      length = len(self.sunVector)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.sunVector))
      _x = self
      buff.write(_struct_2i.pack(_x.errorCode1, _x.errorCode2))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.image = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (self.tempCelcius,) = _struct_f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.sunVector = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 8
      (_x.errorCode1, _x.errorCode2,) = _struct_2i.unpack(str[start:end])
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
      length = len(self.image)
      buff.write(_struct_I.pack(length))
      pattern = '<%sH'%length
      buff.write(self.image.tostring())
      buff.write(_struct_f.pack(self.tempCelcius))
      length = len(self.sunVector)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.sunVector.tostring())
      _x = self
      buff.write(_struct_2i.pack(_x.errorCode1, _x.errorCode2))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sH'%length
      start = end
      end += struct.calcsize(pattern)
      self.image = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=length)
      start = end
      end += 4
      (self.tempCelcius,) = _struct_f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.sunVector = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 8
      (_x.errorCode1, _x.errorCode2,) = _struct_2i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_2i = struct.Struct("<2i")
_struct_f = struct.Struct("<f")
