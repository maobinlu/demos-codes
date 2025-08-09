;; Auto-generated. Do not edit!


(when (boundp 'robo_msgs::Pendulum)
  (if (not (find-package "ROBO_MSGS"))
    (make-package "ROBO_MSGS"))
  (shadow 'Pendulum (find-package "ROBO_MSGS")))
(unless (find-package "ROBO_MSGS::PENDULUM")
  (make-package "ROBO_MSGS::PENDULUM"))

(in-package "ROS")
;;//! \htmlinclude Pendulum.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass robo_msgs::Pendulum
  :super ros::object
  :slots (_pred_dt _pos _flag_y ))

(defmethod robo_msgs::Pendulum
  (:init
   (&key
    ((:pred_dt __pred_dt) 0.0)
    ((:pos __pos) (instance geometry_msgs::Point :init))
    ((:flag_y __flag_y) 0)
    )
   (send-super :init)
   (setq _pred_dt (float __pred_dt))
   (setq _pos __pos)
   (setq _flag_y (round __flag_y))
   self)
  (:pred_dt
   (&optional __pred_dt)
   (if __pred_dt (setq _pred_dt __pred_dt)) _pred_dt)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:flag_y
   (&optional __flag_y)
   (if __flag_y (setq _flag_y __flag_y)) _flag_y)
  (:serialization-length
   ()
   (+
    ;; float64 _pred_dt
    8
    ;; geometry_msgs/Point _pos
    (send _pos :serialization-length)
    ;; int32 _flag_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pred_dt
       (sys::poke _pred_dt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _pos
       (send _pos :serialize s)
     ;; int32 _flag_y
       (write-long _flag_y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pred_dt
     (setq _pred_dt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; int32 _flag_y
     (setq _flag_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robo_msgs::Pendulum :md5sum-) "c3387e721239562d2b53a348be4d1547")
(setf (get robo_msgs::Pendulum :datatype-) "robo_msgs/Pendulum")
(setf (get robo_msgs::Pendulum :definition-)
      "float64 pred_dt
geometry_msgs/Point pos
int32 flag_y
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :robo_msgs/Pendulum "c3387e721239562d2b53a348be4d1547")


