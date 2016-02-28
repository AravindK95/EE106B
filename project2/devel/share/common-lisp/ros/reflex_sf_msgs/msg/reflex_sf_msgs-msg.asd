
(cl:in-package :asdf)

(defsystem "reflex_sf_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SFPose" :depends-on ("_package_SFPose"))
    (:file "_package_SFPose" :depends-on ("_package"))
  ))