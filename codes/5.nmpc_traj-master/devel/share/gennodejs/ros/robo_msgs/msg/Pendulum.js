// Auto-generated. Do not edit!

// (in-package robo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Pendulum {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pred_dt = null;
      this.pos = null;
      this.flag_y = null;
    }
    else {
      if (initObj.hasOwnProperty('pred_dt')) {
        this.pred_dt = initObj.pred_dt
      }
      else {
        this.pred_dt = 0.0;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('flag_y')) {
        this.flag_y = initObj.flag_y
      }
      else {
        this.flag_y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pendulum
    // Serialize message field [pred_dt]
    bufferOffset = _serializer.float64(obj.pred_dt, buffer, bufferOffset);
    // Serialize message field [pos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.pos, buffer, bufferOffset);
    // Serialize message field [flag_y]
    bufferOffset = _serializer.int32(obj.flag_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pendulum
    let len;
    let data = new Pendulum(null);
    // Deserialize message field [pred_dt]
    data.pred_dt = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [flag_y]
    data.flag_y = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robo_msgs/Pendulum';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c3387e721239562d2b53a348be4d1547';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 pred_dt
    geometry_msgs/Point pos
    int32 flag_y
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
    const resolved = new Pendulum(null);
    if (msg.pred_dt !== undefined) {
      resolved.pred_dt = msg.pred_dt;
    }
    else {
      resolved.pred_dt = 0.0
    }

    if (msg.pos !== undefined) {
      resolved.pos = geometry_msgs.msg.Point.Resolve(msg.pos)
    }
    else {
      resolved.pos = new geometry_msgs.msg.Point()
    }

    if (msg.flag_y !== undefined) {
      resolved.flag_y = msg.flag_y;
    }
    else {
      resolved.flag_y = 0
    }

    return resolved;
    }
};

module.exports = Pendulum;
