; Auto-generated. Do not edit!


(cl:in-package mavros-msg)


;//! \htmlinclude Mavlink.msg.html

(cl:defclass <Mavlink> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (len
    :reader len
    :initarg :len
    :type cl:fixnum
    :initform 0)
   (seq
    :reader seq
    :initarg :seq
    :type cl:fixnum
    :initform 0)
   (sysid
    :reader sysid
    :initarg :sysid
    :type cl:fixnum
    :initform 0)
   (compid
    :reader compid
    :initarg :compid
    :type cl:fixnum
    :initform 0)
   (msgid
    :reader msgid
    :initarg :msgid
    :type cl:fixnum
    :initform 0)
   (fromlcm
    :reader fromlcm
    :initarg :fromlcm
    :type cl:boolean
    :initform cl:nil)
   (payload64
    :reader payload64
    :initarg :payload64
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Mavlink (<Mavlink>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mavlink>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mavlink)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-msg:<Mavlink> is deprecated: use mavros-msg:Mavlink instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:header-val is deprecated.  Use mavros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'len-val :lambda-list '(m))
(cl:defmethod len-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:len-val is deprecated.  Use mavros-msg:len instead.")
  (len m))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:seq-val is deprecated.  Use mavros-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'sysid-val :lambda-list '(m))
(cl:defmethod sysid-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:sysid-val is deprecated.  Use mavros-msg:sysid instead.")
  (sysid m))

(cl:ensure-generic-function 'compid-val :lambda-list '(m))
(cl:defmethod compid-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:compid-val is deprecated.  Use mavros-msg:compid instead.")
  (compid m))

(cl:ensure-generic-function 'msgid-val :lambda-list '(m))
(cl:defmethod msgid-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:msgid-val is deprecated.  Use mavros-msg:msgid instead.")
  (msgid m))

(cl:ensure-generic-function 'fromlcm-val :lambda-list '(m))
(cl:defmethod fromlcm-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:fromlcm-val is deprecated.  Use mavros-msg:fromlcm instead.")
  (fromlcm m))

(cl:ensure-generic-function 'payload64-val :lambda-list '(m))
(cl:defmethod payload64-val ((m <Mavlink>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-msg:payload64-val is deprecated.  Use mavros-msg:payload64 instead.")
  (payload64 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mavlink>) ostream)
  "Serializes a message object of type '<Mavlink>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'len)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'compid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msgid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fromlcm) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'payload64))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'payload64))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mavlink>) istream)
  "Deserializes a message object of type '<Mavlink>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'len)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'compid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msgid)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fromlcm) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'payload64) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'payload64)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mavlink>)))
  "Returns string type for a message object of type '<Mavlink>"
  "mavros/Mavlink")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mavlink)))
  "Returns string type for a message object of type 'Mavlink"
  "mavros/Mavlink")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mavlink>)))
  "Returns md5sum for a message object of type '<Mavlink>"
  "3513d96e87d97146fed4643086047ddd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mavlink)))
  "Returns md5sum for a message object of type 'Mavlink"
  "3513d96e87d97146fed4643086047ddd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mavlink>)))
  "Returns full string definition for message of type '<Mavlink>"
  (cl:format cl:nil "Header header~%uint8 len~%uint8 seq~%uint8 sysid~%uint8 compid~%uint8 msgid~%bool fromlcm~%uint64[] payload64~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mavlink)))
  "Returns full string definition for message of type 'Mavlink"
  (cl:format cl:nil "Header header~%uint8 len~%uint8 seq~%uint8 sysid~%uint8 compid~%uint8 msgid~%bool fromlcm~%uint64[] payload64~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mavlink>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'payload64) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mavlink>))
  "Converts a ROS message object to a list"
  (cl:list 'Mavlink
    (cl:cons ':header (header msg))
    (cl:cons ':len (len msg))
    (cl:cons ':seq (seq msg))
    (cl:cons ':sysid (sysid msg))
    (cl:cons ':compid (compid msg))
    (cl:cons ':msgid (msgid msg))
    (cl:cons ':fromlcm (fromlcm msg))
    (cl:cons ':payload64 (payload64 msg))
))
