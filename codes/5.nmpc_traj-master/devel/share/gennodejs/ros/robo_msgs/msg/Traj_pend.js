// Auto-generated. Do not edit!

// (in-package robo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Traj_pend {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.traj_id = null;
      this.pos = null;
      this.yaw = null;
      this.time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('traj_id')) {
        this.traj_id = initObj.traj_id
      }
      else {
        this.traj_id = 0;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = [];
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Traj_pend
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [traj_id]
    bufferOffset = _serializer.uint32(obj.traj_id, buffer, bufferOffset);
    // Serialize message field [pos]
    // Serialize the length for message field [pos]
    bufferOffset = _serializer.uint32(obj.pos.length, buffer, bufferOffset);
    obj.pos.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [yaw]
    bufferOffset = _arraySerializer.float64(obj.yaw, buffer, bufferOffset, null);
    // Serialize message field [time]
    bufferOffset = _arraySerializer.float64(obj.time, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Traj_pend
    let len;
    let data = new Traj_pend(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [traj_id]
    data.traj_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pos]
    // Deserialize array length for message field [pos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pos[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [yaw]
    data.yaw = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [time]
    data.time = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.pos.length;
    length += 8 * object.yaw.length;
    length += 8 * object.time.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robo_msgs/Traj_pend';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac1e32defbaa93c0b56eb1120a69285a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # 轨迹的id，用来标记不同轨迹
    uint32 traj_id
    
    geometry_msgs/Point[] pos # 轨迹位置点
    float64[] yaw # 位置点对应的偏航角
    float64[] time # 位置点对应的时间
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Traj_pend(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.traj_id !== undefined) {
      resolved.traj_id = msg.traj_id;
    }
    else {
      resolved.traj_id = 0
    }

    if (msg.pos !== undefined) {
      resolved.pos = new Array(msg.pos.length);
      for (let i = 0; i < resolved.pos.length; ++i) {
        resolved.pos[i] = geometry_msgs.msg.Point.Resolve(msg.pos[i]);
      }
    }
    else {
      resolved.pos = []
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = []
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = []
    }

    return resolved;
    }
};

module.exports = Traj_pend;
