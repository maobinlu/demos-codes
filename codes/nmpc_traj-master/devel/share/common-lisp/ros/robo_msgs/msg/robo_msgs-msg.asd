
(cl:in-package :asdf)

(defsystem "robo_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Param" :depends-on ("_package_Param"))
    (:file "_package_Param" :depends-on ("_package"))
    (:file "Pendulum" :depends-on ("_package_Pendulum"))
    (:file "_package_Pendulum" :depends-on ("_package"))
    (:file "TrackTraj" :depends-on ("_package_TrackTraj"))
    (:file "_package_TrackTraj" :depends-on ("_package"))
    (:file "Traj_pend" :depends-on ("_package_Traj_pend"))
    (:file "_package_Traj_pend" :depends-on ("_package"))
  ))