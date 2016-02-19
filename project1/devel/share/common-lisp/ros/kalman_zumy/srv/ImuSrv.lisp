; Auto-generated. Do not edit!


(cl:in-package kalman_zumy-srv)


;//! \htmlinclude ImuSrv-request.msg.html

(cl:defclass <ImuSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ImuSrv-request (<ImuSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kalman_zumy-srv:<ImuSrv-request> is deprecated: use kalman_zumy-srv:ImuSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuSrv-request>) ostream)
  "Serializes a message object of type '<ImuSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuSrv-request>) istream)
  "Deserializes a message object of type '<ImuSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuSrv-request>)))
  "Returns string type for a service object of type '<ImuSrv-request>"
  "kalman_zumy/ImuSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSrv-request)))
  "Returns string type for a service object of type 'ImuSrv-request"
  "kalman_zumy/ImuSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuSrv-request>)))
  "Returns md5sum for a message object of type '<ImuSrv-request>"
  "d41085fd9e23edf9efb8c5d896ef9228")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuSrv-request)))
  "Returns md5sum for a message object of type 'ImuSrv-request"
  "d41085fd9e23edf9efb8c5d896ef9228")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuSrv-request>)))
  "Returns full string definition for message of type '<ImuSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuSrv-request)))
  "Returns full string definition for message of type 'ImuSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuSrv-request
))
;//! \htmlinclude ImuSrv-response.msg.html

(cl:defclass <ImuSrv-response> (roslisp-msg-protocol:ros-message)
  ((linear_acceleration
    :reader linear_acceleration
    :initarg :linear_acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (linear_acceleration_filtered
    :reader linear_acceleration_filtered
    :initarg :linear_acceleration_filtered
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular_velocity_filtered
    :reader angular_velocity_filtered
    :initarg :angular_velocity_filtered
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass ImuSrv-response (<ImuSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kalman_zumy-srv:<ImuSrv-response> is deprecated: use kalman_zumy-srv:ImuSrv-response instead.")))

(cl:ensure-generic-function 'linear_acceleration-val :lambda-list '(m))
(cl:defmethod linear_acceleration-val ((m <ImuSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kalman_zumy-srv:linear_acceleration-val is deprecated.  Use kalman_zumy-srv:linear_acceleration instead.")
  (linear_acceleration m))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <ImuSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kalman_zumy-srv:angular_velocity-val is deprecated.  Use kalman_zumy-srv:angular_velocity instead.")
  (angular_velocity m))

(cl:ensure-generic-function 'linear_acceleration_filtered-val :lambda-list '(m))
(cl:defmethod linear_acceleration_filtered-val ((m <ImuSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kalman_zumy-srv:linear_acceleration_filtered-val is deprecated.  Use kalman_zumy-srv:linear_acceleration_filtered instead.")
  (linear_acceleration_filtered m))

(cl:ensure-generic-function 'angular_velocity_filtered-val :lambda-list '(m))
(cl:defmethod angular_velocity_filtered-val ((m <ImuSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kalman_zumy-srv:angular_velocity_filtered-val is deprecated.  Use kalman_zumy-srv:angular_velocity_filtered instead.")
  (angular_velocity_filtered m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuSrv-response>) ostream)
  "Serializes a message object of type '<ImuSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_acceleration_filtered) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_velocity_filtered) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuSrv-response>) istream)
  "Deserializes a message object of type '<ImuSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_acceleration_filtered) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_velocity_filtered) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuSrv-response>)))
  "Returns string type for a service object of type '<ImuSrv-response>"
  "kalman_zumy/ImuSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSrv-response)))
  "Returns string type for a service object of type 'ImuSrv-response"
  "kalman_zumy/ImuSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuSrv-response>)))
  "Returns md5sum for a message object of type '<ImuSrv-response>"
  "d41085fd9e23edf9efb8c5d896ef9228")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuSrv-response)))
  "Returns md5sum for a message object of type 'ImuSrv-response"
  "d41085fd9e23edf9efb8c5d896ef9228")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuSrv-response>)))
  "Returns full string definition for message of type '<ImuSrv-response>"
  (cl:format cl:nil "geometry_msgs/Vector3 linear_acceleration~%geometry_msgs/Vector3 angular_velocity~%geometry_msgs/Vector3 linear_acceleration_filtered~%geometry_msgs/Vector3 angular_velocity_filtered~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuSrv-response)))
  "Returns full string definition for message of type 'ImuSrv-response"
  (cl:format cl:nil "geometry_msgs/Vector3 linear_acceleration~%geometry_msgs/Vector3 angular_velocity~%geometry_msgs/Vector3 linear_acceleration_filtered~%geometry_msgs/Vector3 angular_velocity_filtered~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_acceleration_filtered))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_velocity_filtered))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuSrv-response
    (cl:cons ':linear_acceleration (linear_acceleration msg))
    (cl:cons ':angular_velocity (angular_velocity msg))
    (cl:cons ':linear_acceleration_filtered (linear_acceleration_filtered msg))
    (cl:cons ':angular_velocity_filtered (angular_velocity_filtered msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImuSrv)))
  'ImuSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImuSrv)))
  'ImuSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSrv)))
  "Returns string type for a service object of type '<ImuSrv>"
  "kalman_zumy/ImuSrv")