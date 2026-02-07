; Auto-generated. Do not edit!


(cl:in-package airsim_ros-msg)


;//! \htmlinclude Predloop.msg.html

(cl:defclass <Predloop> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (loop_flag
    :reader loop_flag
    :initarg :loop_flag
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass Predloop (<Predloop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Predloop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Predloop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name airsim_ros-msg:<Predloop> is deprecated: use airsim_ros-msg:Predloop instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Predloop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-msg:position-val is deprecated.  Use airsim_ros-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'loop_flag-val :lambda-list '(m))
(cl:defmethod loop_flag-val ((m <Predloop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-msg:loop_flag-val is deprecated.  Use airsim_ros-msg:loop_flag instead.")
  (loop_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Predloop>) ostream)
  "Serializes a message object of type '<Predloop>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'loop_flag) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Predloop>) istream)
  "Deserializes a message object of type '<Predloop>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'loop_flag) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Predloop>)))
  "Returns string type for a message object of type '<Predloop>"
  "airsim_ros/Predloop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Predloop)))
  "Returns string type for a message object of type 'Predloop"
  "airsim_ros/Predloop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Predloop>)))
  "Returns md5sum for a message object of type '<Predloop>"
  "8511114e157c218c8b9df16423dc9a97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Predloop)))
  "Returns md5sum for a message object of type 'Predloop"
  "8511114e157c218c8b9df16423dc9a97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Predloop>)))
  "Returns full string definition for message of type '<Predloop>"
  (cl:format cl:nil "geometry_msgs/Pose position~%std_msgs/Bool loop_flag~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Predloop)))
  "Returns full string definition for message of type 'Predloop"
  (cl:format cl:nil "geometry_msgs/Pose position~%std_msgs/Bool loop_flag~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Predloop>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'loop_flag))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Predloop>))
  "Converts a ROS message object to a list"
  (cl:list 'Predloop
    (cl:cons ':position (position msg))
    (cl:cons ':loop_flag (loop_flag msg))
))
