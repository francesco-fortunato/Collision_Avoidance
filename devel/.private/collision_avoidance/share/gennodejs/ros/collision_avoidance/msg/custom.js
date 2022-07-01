// Auto-generated. Do not edit!

// (in-package collision_avoidance.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class custom {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.started = null;
      this.linaer_velocity = null;
      this.angular_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('started')) {
        this.started = initObj.started
      }
      else {
        this.started = false;
      }
      if (initObj.hasOwnProperty('linaer_velocity')) {
        this.linaer_velocity = initObj.linaer_velocity
      }
      else {
        this.linaer_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type custom
    // Serialize message field [started]
    bufferOffset = _serializer.bool(obj.started, buffer, bufferOffset);
    // Serialize message field [linaer_velocity]
    bufferOffset = _serializer.float32(obj.linaer_velocity, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = _serializer.float32(obj.angular_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type custom
    let len;
    let data = new custom(null);
    // Deserialize message field [started]
    data.started = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [linaer_velocity]
    data.linaer_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'collision_avoidance/custom';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c45fdadcf7f0ff78418b31fecc804fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool started
    float32 linaer_velocity
    float32 angular_velocity
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new custom(null);
    if (msg.started !== undefined) {
      resolved.started = msg.started;
    }
    else {
      resolved.started = false
    }

    if (msg.linaer_velocity !== undefined) {
      resolved.linaer_velocity = msg.linaer_velocity;
    }
    else {
      resolved.linaer_velocity = 0.0
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = msg.angular_velocity;
    }
    else {
      resolved.angular_velocity = 0.0
    }

    return resolved;
    }
};

module.exports = custom;
