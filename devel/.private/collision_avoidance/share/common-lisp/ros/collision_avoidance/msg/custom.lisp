; Auto-generated. Do not edit!


(cl:in-package collision_avoidance-msg)


;//! \htmlinclude custom.msg.html

(cl:defclass <custom> (roslisp-msg-protocol:ros-message)
  ((started
    :reader started
    :initarg :started
    :type cl:boolean
    :initform cl:nil)
   (linaer_velocity
    :reader linaer_velocity
    :initarg :linaer_velocity
    :type cl:float
    :initform 0.0)
   (angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass custom (<custom>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <custom>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'custom)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collision_avoidance-msg:<custom> is deprecated: use collision_avoidance-msg:custom instead.")))

(cl:ensure-generic-function 'started-val :lambda-list '(m))
(cl:defmethod started-val ((m <custom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-msg:started-val is deprecated.  Use collision_avoidance-msg:started instead.")
  (started m))

(cl:ensure-generic-function 'linaer_velocity-val :lambda-list '(m))
(cl:defmethod linaer_velocity-val ((m <custom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-msg:linaer_velocity-val is deprecated.  Use collision_avoidance-msg:linaer_velocity instead.")
  (linaer_velocity m))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <custom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-msg:angular_velocity-val is deprecated.  Use collision_avoidance-msg:angular_velocity instead.")
  (angular_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <custom>) ostream)
  "Serializes a message object of type '<custom>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'started) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linaer_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <custom>) istream)
  "Deserializes a message object of type '<custom>"
    (cl:setf (cl:slot-value msg 'started) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linaer_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<custom>)))
  "Returns string type for a message object of type '<custom>"
  "collision_avoidance/custom")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'custom)))
  "Returns string type for a message object of type 'custom"
  "collision_avoidance/custom")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<custom>)))
  "Returns md5sum for a message object of type '<custom>"
  "0c45fdadcf7f0ff78418b31fecc804fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'custom)))
  "Returns md5sum for a message object of type 'custom"
  "0c45fdadcf7f0ff78418b31fecc804fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<custom>)))
  "Returns full string definition for message of type '<custom>"
  (cl:format cl:nil "bool started~%float32 linaer_velocity~%float32 angular_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'custom)))
  "Returns full string definition for message of type 'custom"
  (cl:format cl:nil "bool started~%float32 linaer_velocity~%float32 angular_velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <custom>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <custom>))
  "Converts a ROS message object to a list"
  (cl:list 'custom
    (cl:cons ':started (started msg))
    (cl:cons ':linaer_velocity (linaer_velocity msg))
    (cl:cons ':angular_velocity (angular_velocity msg))
))
