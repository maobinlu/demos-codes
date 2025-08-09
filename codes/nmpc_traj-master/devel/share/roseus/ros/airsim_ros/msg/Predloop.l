;; Auto-generated. Do not edit!


(when (boundp 'airsim_ros::Predloop)
  (if (not (find-package "AIRSIM_ROS"))
    (make-package "AIRSIM_ROS"))
  (shadow 'Predloop (find-package "AIRSIM_ROS")))
(unless (find-package "AIRSIM_ROS::PREDLOOP")
  (make-package "AIRSIM_ROS::PREDLOOP"))

(in-package "ROS")
;;//! \htmlinclude Predloop.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass airsim_ros::Predloop
  :super ros::object
  :slots (_position _loop_flag ))

(defmethod airsim_ros::Predloop
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Pose :init))
    ((:loop_flag __loop_flag) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _loop_flag __loop_flag)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:loop_flag
   (&rest __loop_flag)
   (if (keywordp (car __loop_flag))
       (send* _loop_flag __loop_flag)
     (progn
       (if __loop_flag (setq _loop_flag (car __loop_flag)))
       _loop_flag)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _position
    (send _position :serialization-length)
    ;; std_msgs/Bool _loop_flag
    (send _loop_flag :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _position
       (send _position :serialize s)
     ;; std_msgs/Bool _loop_flag
       (send _loop_flag :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; std_msgs/Bool _loop_flag
     (send _loop_flag :deserialize buf ptr-) (incf ptr- (send _loop_flag :serialization-length))
   ;;
   self)
  )

(setf (get airsim_ros::Predloop :md5sum-) "8511114e157c218c8b9df16423dc9a97")
(setf (get airsim_ros::Predloop :datatype-) "airsim_ros/Predloop")
(setf (get airsim_ros::Predloop :definition-)
      "geometry_msgs/Pose position
std_msgs/Bool loop_flag
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
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

================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :airsim_ros/Predloop "8511114e157c218c8b9df16423dc9a97")


