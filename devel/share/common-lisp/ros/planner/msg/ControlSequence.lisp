; Auto-generated. Do not edit!


(cl:in-package planner-msg)


;//! \htmlinclude ControlSequence.msg.html

(cl:defclass <ControlSequence> (roslisp-msg-protocol:ros-message)
  ((controls
    :reader controls
    :initarg :controls
    :type (cl:vector planner-msg:Control)
   :initform (cl:make-array 0 :element-type 'planner-msg:Control :initial-element (cl:make-instance 'planner-msg:Control))))
)

(cl:defclass ControlSequence (<ControlSequence>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlSequence>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlSequence)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-msg:<ControlSequence> is deprecated: use planner-msg:ControlSequence instead.")))

(cl:ensure-generic-function 'controls-val :lambda-list '(m))
(cl:defmethod controls-val ((m <ControlSequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:controls-val is deprecated.  Use planner-msg:controls instead.")
  (controls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlSequence>) ostream)
  "Serializes a message object of type '<ControlSequence>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controls))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controls))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlSequence>) istream)
  "Deserializes a message object of type '<ControlSequence>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controls) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controls)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'planner-msg:Control))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlSequence>)))
  "Returns string type for a message object of type '<ControlSequence>"
  "planner/ControlSequence")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlSequence)))
  "Returns string type for a message object of type 'ControlSequence"
  "planner/ControlSequence")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlSequence>)))
  "Returns md5sum for a message object of type '<ControlSequence>"
  "7b73a45fc9f216f397bef7a4afdff8b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlSequence)))
  "Returns md5sum for a message object of type 'ControlSequence"
  "7b73a45fc9f216f397bef7a4afdff8b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlSequence>)))
  "Returns full string definition for message of type '<ControlSequence>"
  (cl:format cl:nil "Control[] controls~%================================================================================~%MSG: planner/Control~%float64 v~%float64 w~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlSequence)))
  "Returns full string definition for message of type 'ControlSequence"
  (cl:format cl:nil "Control[] controls~%================================================================================~%MSG: planner/Control~%float64 v~%float64 w~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlSequence>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controls) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlSequence>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlSequence
    (cl:cons ':controls (controls msg))
))
