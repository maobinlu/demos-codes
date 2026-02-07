;; Auto-generated. Do not edit!


(when (boundp 'robo_msgs::Traj_pend)
  (if (not (find-package "ROBO_MSGS"))
    (make-package "ROBO_MSGS"))
  (shadow 'Traj_pend (find-package "ROBO_MSGS")))
(unless (find-package "ROBO_MSGS::TRAJ_PEND")
  (make-package "ROBO_MSGS::TRAJ_PEND"))

(in-package "ROS")
;;//! \htmlinclude Traj_pend.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robo_msgs::Traj_pend
  :super ros::object
  :slots (_header _traj_id _pos _yaw _time ))

(defmethod robo_msgs::Traj_pend
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:traj_id __traj_id) 0)
    ((:pos __pos) ())
    ((:yaw __yaw) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time __time) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _traj_id (round __traj_id))
   (setq _pos __pos)
   (setq _yaw __yaw)
   (setq _time __time)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:traj_id
   (&optional __traj_id)
   (if __traj_id (setq _traj_id __traj_id)) _traj_id)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _traj_id
    4
    ;; geometry_msgs/Point[] _pos
    (apply #'+ (send-all _pos :serialization-length)) 4
    ;; float64[] _yaw
    (* 8    (length _yaw)) 4
    ;; float64[] _time
    (* 8    (length _time)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _traj_id
       (write-long _traj_id s)
     ;; geometry_msgs/Point[] _pos
     (write-long (length _pos) s)
     (dolist (elem _pos)
       (send elem :serialize s)
       )
     ;; float64[] _yaw
     (write-long (length _yaw) s)
     (dotimes (i (length _yaw))
       (sys::poke (elt _yaw i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _time
     (write-long (length _time) s)
     (dotimes (i (length _time))
       (sys::poke (elt _time i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _traj_id
     (setq _traj_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _yaw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yaw (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _yaw i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _time
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get robo_msgs::Traj_pend :md5sum-) "ac1e32defbaa93c0b56eb1120a69285a")
(setf (get robo_msgs::Traj_pend :datatype-) "robo_msgs/Traj_pend")
(setf (get robo_msgs::Traj_pend :definition-)
      "Header header

# 轨迹的id，用来标记不同轨迹
uint32 traj_id

geometry_msgs/Point[] pos # 轨迹位置点
float64[] yaw # 位置点对应的偏航角
float64[] time # 位置点对应的时间

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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :robo_msgs/Traj_pend "ac1e32defbaa93c0b56eb1120a69285a")


