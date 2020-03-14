// Auto-generated. Do not edit!

// (in-package planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Control = require('./Control.js');

//-----------------------------------------------------------

class ControlSequence {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controls = null;
    }
    else {
      if (initObj.hasOwnProperty('controls')) {
        this.controls = initObj.controls
      }
      else {
        this.controls = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlSequence
    // Serialize message field [controls]
    // Serialize the length for message field [controls]
    bufferOffset = _serializer.uint32(obj.controls.length, buffer, bufferOffset);
    obj.controls.forEach((val) => {
      bufferOffset = Control.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlSequence
    let len;
    let data = new ControlSequence(null);
    // Deserialize message field [controls]
    // Deserialize array length for message field [controls]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controls = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controls[i] = Control.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.controls.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'planner/ControlSequence';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7b73a45fc9f216f397bef7a4afdff8b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Control[] controls
    ================================================================================
    MSG: planner/Control
    float64 v
    float64 w
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlSequence(null);
    if (msg.controls !== undefined) {
      resolved.controls = new Array(msg.controls.length);
      for (let i = 0; i < resolved.controls.length; ++i) {
        resolved.controls[i] = Control.Resolve(msg.controls[i]);
      }
    }
    else {
      resolved.controls = []
    }

    return resolved;
    }
};

module.exports = ControlSequence;
