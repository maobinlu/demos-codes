;; Auto-generated. Do not edit!


(when (boundp 'robo_msgs::Param)
  (if (not (find-package "ROBO_MSGS"))
    (make-package "ROBO_MSGS"))
  (shadow 'Param (find-package "ROBO_MSGS")))
(unless (find-package "ROBO_MSGS::PARAM")
  (make-package "ROBO_MSGS::PARAM"))

(in-package "ROS")
;;//! \htmlinclude Param.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robo_msgs::Param
  :super ros::object
  :slots (_header _param_y _param_z _t_next ))

(defmethod robo_msgs::Param
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:param_y __param_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:param_z __param_z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:t_next __t_next) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _param_y __param_y)
   (setq _param_z __param_z)
   (setq _t_next (float __t_next))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:param_y
   (&optional __param_y)
   (if __param_y (setq _param_y __param_y)) _param_y)
  (:param_z
   (&optional __param_z)
   (if __param_z (setq _param_z __param_z)) _param_z)
  (:t_next
   (&optional __t_next)
   (if __t_next (setq _t_next __t_next)) _t_next)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _param_y
    (* 8    (length _param_y)) 4
    ;; float64[] _param_z
    (* 8    (length _param_z)) 4
    ;; float64 _t_next
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _param_y
     (write-long (length _param_y) s)
     (dotimes (i (length _param_y))
       (sys::poke (elt _param_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _param_z
     (write-long (length _param_z) s)
     (dotimes (i (length _param_z))
       (sys::poke (elt _param_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _t_next
       (sys::poke _t_next (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _param_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _param_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _param_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _param_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _param_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _param_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _t_next
     (setq _t_next (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robo_msgs::Param :md5sum-) "b9d15ed2b648eaa21ef0207f8919cba2")
(setf (get robo_msgs::Param :datatype-) "robo_msgs/Param")
(setf (get robo_msgs::Param :definition-)
      "Header header
float64[] param_y
float64[] param_z
float64 t_next



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :robo_msgs/Param "b9d15ed2b648eaa21ef0207f8919cba2")


