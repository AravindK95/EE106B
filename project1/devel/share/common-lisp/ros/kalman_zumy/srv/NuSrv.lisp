; Auto-generated. Do not edit!


(cl:in-package kalman_zumy-srv)


;//! \htmlinclude NuSrv-request.msg.html

(cl:defclass <NuSrv-request> (roslisp-msg-protocol:ros-message)
  ((transform
    :reader transform
    :initarg :transform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (origin_tag
    :reader origin_tag
    :initarg :origin_tag
    :type cl:string
    :initform ""))
)

(cl:defclass NuSrv-request (<NuSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NuSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NuSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kalman_zumy-srv:<NuSrv-request> is deprecated: use kalman_zumy-srv:NuSrv-request instead.")))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <NuSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kalman_zumy-srv:transform-val is deprecated.  Use kalman_zumy-srv:transform instead.")
  (transform m))

(cl:ensure-generic-function 'origin_tag-val :lambda-list '(m))
(cl:defmethod origin_tag-val ((m <NuSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kalman_zumy-srv:origin_tag-val is deprecated.  Use kalman_zumy-srv:origin_tag instead.")
  (origin_tag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NuSrv-request>) ostream)
  "Serializes a message object of type '<NuSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transform) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'origin_tag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'origin_tag))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NuSrv-request>) istream)
  "Deserializes a message object of type '<NuSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transform) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'origin_tag) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'origin_tag) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NuSrv-request>)))
  "Returns string type for a service object of type '<NuSrv-request>"
  "kalman_zumy/NuSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NuSrv-request)))
  "Returns string type for a service object of type 'NuSrv-request"
  "kalman_zumy/NuSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NuSrv-request>)))
  "Returns md5sum for a message object of type '<NuSrv-request>"
  "a8d9d17b37b0c9a4256566443542d8f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NuSrv-request)))
  "Returns md5sum for a message object of type 'NuSrv-request"
  "a8d9d17b37b0c9a4256566443542d8f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NuSrv-request>)))
  "Returns full string definition for message of type '<NuSrv-request>"
  (cl:format cl:nil "geometry_msgs/Transform transform~%string origin_tag~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NuSrv-request)))
  "Returns full string definition for message of type 'NuSrv-request"
  (cl:format cl:nil "geometry_msgs/Transform transform~%string origin_tag~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NuSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transform))
     4 (cl:length (cl:slot-value msg 'origin_tag))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NuSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NuSrv-request
    (cl:cons ':transform (transform msg))
    (cl:cons ':origin_tag (origin_tag msg))
))
;//! \htmlinclude NuSrv-response.msg.html

(cl:defclass <NuSrv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NuSrv-response (<NuSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NuSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NuSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kalman_zumy-srv:<NuSrv-response> is deprecated: use kalman_zumy-srv:NuSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NuSrv-response>) ostream)
  "Serializes a message object of type '<NuSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NuSrv-response>) istream)
  "Deserializes a message object of type '<NuSrv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NuSrv-response>)))
  "Returns string type for a service object of type '<NuSrv-response>"
  "kalman_zumy/NuSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NuSrv-response)))
  "Returns string type for a service object of type 'NuSrv-response"
  "kalman_zumy/NuSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NuSrv-response>)))
  "Returns md5sum for a message object of type '<NuSrv-response>"
  "a8d9d17b37b0c9a4256566443542d8f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NuSrv-response)))
  "Returns md5sum for a message object of type 'NuSrv-response"
  "a8d9d17b37b0c9a4256566443542d8f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NuSrv-response>)))
  "Returns full string definition for message of type '<NuSrv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NuSrv-response)))
  "Returns full string definition for message of type 'NuSrv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NuSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NuSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NuSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NuSrv)))
  'NuSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NuSrv)))
  'NuSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NuSrv)))
  "Returns string type for a service object of type '<NuSrv>"
  "kalman_zumy/NuSrv")