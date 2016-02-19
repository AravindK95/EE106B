
(cl:in-package :asdf)

(defsystem "kalman_zumy-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ImuSrv" :depends-on ("_package_ImuSrv"))
    (:file "_package_ImuSrv" :depends-on ("_package"))
    (:file "NuSrv" :depends-on ("_package_NuSrv"))
    (:file "_package_NuSrv" :depends-on ("_package"))
  ))