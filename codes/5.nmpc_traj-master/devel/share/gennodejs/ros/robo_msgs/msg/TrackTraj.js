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

class TrackTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.start_time = null;
      this.dt = null;
      this.position = null;
      this.velocity = null;
      this.orientation = null;
      this.angular = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('dt')) {
        this.dt = initObj.dt
      }
      else {
        this.dt = [];
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = [];
      }
      if (initObj.hasOwnProperty('angular')) {
        this.angular = initObj.angular
      }
      else {
        this.angular = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackTraj
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int32(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [dt]
    bufferOffset = _arraySerializer.float64(obj.dt, buffer, bufferOffset, null);
    // Serialize message field [position]
    // Serialize the length for message field [position]
    bufferOffset = _serializer.uint32(obj.position.length, buffer, bufferOffset);
    obj.position.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [velocity]
    // Serialize the length for message field [velocity]
    bufferOffset = _serializer.uint32(obj.velocity.length, buffer, bufferOffset);
    obj.velocity.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [orientation]
    // Serialize the length for message field [orientation]
    bufferOffset = _serializer.uint32(obj.orientation.length, buffer, bufferOffset);
    obj.orientation.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Quaternion.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [angular]
    // Serialize the length for message field [angular]
    bufferOffset = _serializer.uint32(obj.angular.length, buffer, bufferOffset);
    obj.angular.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackTraj
    let len;
    let data = new TrackTraj(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [dt]
    data.dt = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [position]
    // Deserialize array length for message field [position]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.position = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.position[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [velocity]
    // Deserialize array length for message field [velocity]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.velocity = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.velocity[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [orientation]
    // Deserialize array length for message field [orientation]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.orientation = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.orientation[i] = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [angular]
    // Deserialize array length for message field [angular]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.angular = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.angular[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.dt.length;
    length += 24 * object.position.length;
    length += 24 * object.velocity.length;
    length += 32 * object.orientation.length;
    length += 24 * object.angular.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robo_msgs/TrackTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4fa02baf9e303d78b3ed2aba372959e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 drone_id
    time start_time
    
    float64[] dt
    geometry_msgs/Point[] position
    geometry_msgs/Vector3[] velocity
    geometry_msgs/Quaternion[] orientation
    geometry_msgs/Vector3[] angular
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackTraj(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.dt !== undefined) {
      resolved.dt = msg.dt;
    }
    else {
      resolved.dt = []
    }

    if (msg.position !== undefined) {
      resolved.position = new Array(msg.position.length);
      for (let i = 0; i < resolved.position.length; ++i) {
        resolved.position[i] = geometry_msgs.msg.Point.Resolve(msg.position[i]);
      }
    }
    else {
      resolved.position = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = new Array(msg.velocity.length);
      for (let i = 0; i < resolved.velocity.length; ++i) {
        resolved.velocity[i] = geometry_msgs.msg.Vector3.Resolve(msg.velocity[i]);
      }
    }
    else {
      resolved.velocity = []
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = new Array(msg.orientation.length);
      for (let i = 0; i < resolved.orientation.length; ++i) {
        resolved.orientation[i] = geometry_msgs.msg.Quaternion.Resolve(msg.orientation[i]);
      }
    }
    else {
      resolved.orientation = []
    }

    if (msg.angular !== undefined) {
      resolved.angular = new Array(msg.angular.length);
      for (let i = 0; i < resolved.angular.length; ++i) {
        resolved.angular[i] = geometry_msgs.msg.Vector3.Resolve(msg.angular[i]);
      }
    }
    else {
      resolved.angular = []
    }

    return resolved;
    }
};

module.exports = TrackTraj;
