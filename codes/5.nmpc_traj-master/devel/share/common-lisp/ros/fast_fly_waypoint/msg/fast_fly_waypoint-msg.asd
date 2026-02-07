
(cl:in-package :asdf)

(defsystem "fast_fly_waypoint-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "TrackTraj" :depends-on ("_package_TrackTraj"))
    (:file "_package_TrackTraj" :depends-on ("_package"))
  ))