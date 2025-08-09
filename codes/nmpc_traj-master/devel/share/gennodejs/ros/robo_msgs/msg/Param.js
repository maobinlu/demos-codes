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

//-----------------------------------------------------------

class Param {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.param_y = null;
      this.param_z = null;
      this.t_next = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('param_y')) {
        this.param_y = initObj.param_y
      }
      else {
        this.param_y = [];
      }
      if (initObj.hasOwnProperty('param_z')) {
        this.param_z = initObj.param_z
      }
      else {
        this.param_z = [];
      }
      if (initObj.hasOwnProperty('t_next')) {
        this.t_next = initObj.t_next
      }
      else {
        this.t_next = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Param
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [param_y]
    bufferOffset = _arraySerializer.float64(obj.param_y, buffer, bufferOffset, null);
    // Serialize message field [param_z]
    bufferOffset = _arraySerializer.float64(obj.param_z, buffer, bufferOffset, null);
    // Serialize message field [t_next]
    bufferOffset = _serializer.float64(obj.t_next, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Param
    let len;
    let data = new Param(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [param_y]
    data.param_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [param_z]
    data.param_z = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [t_next]
    data.t_next = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.param_y.length;
    length += 8 * object.param_z.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robo_msgs/Param';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b9d15ed2b648eaa21ef0207f8919cba2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64[] param_y
    float64[] param_z
    float64 t_next
    
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Param(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.param_y !== undefined) {
      resolved.param_y = msg.param_y;
    }
    else {
      resolved.param_y = []
    }

    if (msg.param_z !== undefined) {
      resolved.param_z = msg.param_z;
    }
    else {
      resolved.param_z = []
    }

    if (msg.t_next !== undefined) {
      resolved.t_next = msg.t_next;
    }
    else {
      resolved.t_next = 0.0
    }

    return resolved;
    }
};

module.exports = Param;
