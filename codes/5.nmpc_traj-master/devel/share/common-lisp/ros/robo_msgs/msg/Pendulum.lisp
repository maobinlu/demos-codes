; Auto-generated. Do not edit!


(cl:in-package robo_msgs-msg)


;//! \htmlinclude Pendulum.msg.html

(cl:defclass <Pendulum> (roslisp-msg-protocol:ros-message)
  ((pred_dt
    :reader pred_dt
    :initarg :pred_dt
    :type cl:float
    :initform 0.0)
   (pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (flag_y
    :reader flag_y
    :initarg :flag_y
    :type cl:integer
    :initform 0))
)

(cl:defclass Pendulum (<Pendulum>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pendulum>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pendulum)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robo_msgs-msg:<Pendulum> is deprecated: use robo_msgs-msg:Pendulum instead.")))

(cl:ensure-generic-function 'pred_dt-val :lambda-list '(m))
(cl:defmethod pred_dt-val ((m <Pendulum>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robo_msgs-msg:pred_dt-val is deprecated.  Use robo_msgs-msg:pred_dt instead.")
  (pred_dt m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Pendulum>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robo_msgs-msg:pos-val is deprecated.  Use robo_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'flag_y-val :lambda-list '(m))
(cl:defmethod flag_y-val ((m <Pendulum>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robo_msgs-msg:flag_y-val is deprecated.  Use robo_msgs-msg:flag_y instead.")
  (flag_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pendulum>) ostream)
  "Serializes a message object of type '<Pendulum>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pred_dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:let* ((signed (cl:slot-value msg 'flag_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pendulum>) istream)
  "Deserializes a message object of type '<Pendulum>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pred_dt) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flag_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pendulum>)))
  "Returns string type for a message object of type '<Pendulum>"
  "robo_msgs/Pendulum")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pendulum)))
  "Returns string type for a message object of type 'Pendulum"
  "robo_msgs/Pendulum")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pendulum>)))
  "Returns md5sum for a message object of type '<Pendulum>"
  "c3387e721239562d2b53a348be4d1547")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pendulum)))
  "Returns md5sum for a message object of type 'Pendulum"
  "c3387e721239562d2b53a348be4d1547")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pendulum>)))
  "Returns full string definition for message of type '<Pendulum>"
  (cl:format cl:nil "float64 pred_dt~%geometry_msgs/Point pos~%int32 flag_y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pendulum)))
  "Returns full string definition for message of type 'Pendulum"
  (cl:format cl:nil "float64 pred_dt~%geometry_msgs/Point pos~%int32 flag_y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pendulum>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pendulum>))
  "Converts a ROS message object to a list"
  (cl:list 'Pendulum
    (cl:cons ':pred_dt (pred_dt msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':flag_y (flag_y msg))
))
