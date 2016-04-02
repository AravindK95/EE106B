; Auto-generated. Do not edit!


(cl:in-package lab3-msg)


;//! \htmlinclude FrameCall.msg.html

(cl:defclass <FrameCall> (roslisp-msg-protocol:ros-message)
  ((rbt
    :reader rbt
    :initarg :rbt
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (to_add
    :reader to_add
    :initarg :to_add
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FrameCall (<FrameCall>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrameCall>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrameCall)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab3-msg:<FrameCall> is deprecated: use lab3-msg:FrameCall instead.")))

(cl:ensure-generic-function 'rbt-val :lambda-list '(m))
(cl:defmethod rbt-val ((m <FrameCall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab3-msg:rbt-val is deprecated.  Use lab3-msg:rbt instead.")
  (rbt m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <FrameCall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab3-msg:name-val is deprecated.  Use lab3-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'to_add-val :lambda-list '(m))
(cl:defmethod to_add-val ((m <FrameCall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab3-msg:to_add-val is deprecated.  Use lab3-msg:to_add instead.")
  (to_add m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrameCall>) ostream)
  "Serializes a message object of type '<FrameCall>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rbt) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'to_add) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrameCall>) istream)
  "Deserializes a message object of type '<FrameCall>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rbt) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'to_add) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrameCall>)))
  "Returns string type for a message object of type '<FrameCall>"
  "lab3/FrameCall")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrameCall)))
  "Returns string type for a message object of type 'FrameCall"
  "lab3/FrameCall")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrameCall>)))
  "Returns md5sum for a message object of type '<FrameCall>"
  "dd65101815e67ba4d73d834625689ad5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrameCall)))
  "Returns md5sum for a message object of type 'FrameCall"
  "dd65101815e67ba4d73d834625689ad5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrameCall>)))
  "Returns full string definition for message of type '<FrameCall>"
  (cl:format cl:nil "geometry_msgs/Transform rbt~%string name~%bool to_add~%~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrameCall)))
  "Returns full string definition for message of type 'FrameCall"
  (cl:format cl:nil "geometry_msgs/Transform rbt~%string name~%bool to_add~%~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrameCall>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rbt))
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrameCall>))
  "Converts a ROS message object to a list"
  (cl:list 'FrameCall
    (cl:cons ':rbt (rbt msg))
    (cl:cons ':name (name msg))
    (cl:cons ':to_add (to_add msg))
))
