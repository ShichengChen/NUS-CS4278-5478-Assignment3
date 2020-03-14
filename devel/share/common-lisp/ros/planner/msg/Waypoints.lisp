; Auto-generated. Do not edit!


(cl:in-package planner-msg)


;//! \htmlinclude Waypoints.msg.html

(cl:defclass <Waypoints> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector planner-msg:Point)
   :initform (cl:make-array 0 :element-type 'planner-msg:Point :initial-element (cl:make-instance 'planner-msg:Point))))
)

(cl:defclass Waypoints (<Waypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Waypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Waypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-msg:<Waypoints> is deprecated: use planner-msg:Waypoints instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:points-val is deprecated.  Use planner-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Waypoints>) ostream)
  "Serializes a message object of type '<Waypoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Waypoints>) istream)
  "Deserializes a message object of type '<Waypoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'planner-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Waypoints>)))
  "Returns string type for a message object of type '<Waypoints>"
  "planner/Waypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Waypoints)))
  "Returns string type for a message object of type 'Waypoints"
  "planner/Waypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Waypoints>)))
  "Returns md5sum for a message object of type '<Waypoints>"
  "8f02263beef99aa03117a577a3eb879d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Waypoints)))
  "Returns md5sum for a message object of type 'Waypoints"
  "8f02263beef99aa03117a577a3eb879d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Waypoints>)))
  "Returns full string definition for message of type '<Waypoints>"
  (cl:format cl:nil "Point[] points~%================================================================================~%MSG: planner/Point~%float64 x~%float64 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Waypoints)))
  "Returns full string definition for message of type 'Waypoints"
  (cl:format cl:nil "Point[] points~%================================================================================~%MSG: planner/Point~%float64 x~%float64 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Waypoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Waypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'Waypoints
    (cl:cons ':points (points msg))
))
