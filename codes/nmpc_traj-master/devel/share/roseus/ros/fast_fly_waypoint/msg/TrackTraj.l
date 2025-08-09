;; Auto-generated. Do not edit!


(when (boundp 'fast_fly_waypoint::TrackTraj)
  (if (not (find-package "FAST_FLY_WAYPOINT"))
    (make-package "FAST_FLY_WAYPOINT"))
  (shadow 'TrackTraj (find-package "FAST_FLY_WAYPOINT")))
(unless (find-package "FAST_FLY_WAYPOINT::TRACKTRAJ")
  (make-package "FAST_FLY_WAYPOINT::TRACKTRAJ"))

(in-package "ROS")
;;//! \htmlinclude TrackTraj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass fast_fly_waypoint::TrackTraj
  :super ros::object
  :slots (_drone_id _start_time _dt _position _velocity _orientation _angular ))

(defmethod fast_fly_waypoint::TrackTraj
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:dt __dt) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:position __position) ())
    ((:velocity __velocity) ())
    ((:orientation __orientation) ())
    ((:angular __angular) ())
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _start_time __start_time)
   (setq _dt __dt)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _orientation __orientation)
   (setq _angular __angular)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:angular
   (&rest __angular)
   (if (keywordp (car __angular))
       (send* _angular __angular)
     (progn
       (if __angular (setq _angular (car __angular)))
       _angular)))
  (:serialization-length
   ()
   (+
    ;; int32 _drone_id
    4
    ;; time _start_time
    8
    ;; float64[] _dt
    (* 8    (length _dt)) 4
    ;; geometry_msgs/Point[] _position
    (apply #'+ (send-all _position :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _velocity
    (apply #'+ (send-all _velocity :serialization-length)) 4
    ;; geometry_msgs/Quaternion[] _orientation
    (apply #'+ (send-all _orientation :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _angular
    (apply #'+ (send-all _angular :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_id
       (write-long _drone_id s)
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; float64[] _dt
     (write-long (length _dt) s)
     (dotimes (i (length _dt))
       (sys::poke (elt _dt i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Point[] _position
     (write-long (length _position) s)
     (dolist (elem _position)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _velocity
     (write-long (length _velocity) s)
     (dolist (elem _velocity)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Quaternion[] _orientation
     (write-long (length _orientation) s)
     (dolist (elem _orientation)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _angular
     (write-long (length _angular) s)
     (dolist (elem _angular)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_id
     (setq _drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _dt
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dt (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dt i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Point[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _position)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _velocity)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Quaternion[] _orientation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _orientation (let (r) (dotimes (i n) (push (instance geometry_msgs::Quaternion :init) r)) r))
     (dolist (elem- _orientation)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _angular
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angular (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _angular)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fast_fly_waypoint::TrackTraj :md5sum-) "a4fa02baf9e303d78b3ed2aba372959e")
(setf (get fast_fly_waypoint::TrackTraj :datatype-) "fast_fly_waypoint/TrackTraj")
(setf (get fast_fly_waypoint::TrackTraj :definition-)
      "int32 drone_id
time start_time

float64[] dt
geometry_msgs/Point[] position
geometry_msgs/Vector3[] velocity
geometry_msgs/Quaternion[] orientation
geometry_msgs/Vector3[] angular

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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

")



(provide :fast_fly_waypoint/TrackTraj "a4fa02baf9e303d78b3ed2aba372959e")


