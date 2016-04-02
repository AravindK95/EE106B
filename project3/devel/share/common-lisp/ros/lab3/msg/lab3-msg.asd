
(cl:in-package :asdf)

(defsystem "lab3-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FrameCall" :depends-on ("_package_FrameCall"))
    (:file "_package_FrameCall" :depends-on ("_package"))
  ))