; Auto-generated. Do not edit!


(cl:in-package mavros-srv)


;//! \htmlinclude ParamSet-request.msg.html

(cl:defclass <ParamSet-request> (roslisp-msg-protocol:ros-message)
  ((param_id
    :reader param_id
    :initarg :param_id
    :type cl:string
    :initform "")
   (integer
    :reader integer
    :initarg :integer
    :type cl:integer
    :initform 0)
   (real
    :reader real
    :initarg :real
    :type cl:float
    :initform 0.0))
)

(cl:defclass ParamSet-request (<ParamSet-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamSet-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamSet-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<ParamSet-request> is deprecated: use mavros-srv:ParamSet-request instead.")))

(cl:ensure-generic-function 'param_id-val :lambda-list '(m))
(cl:defmethod param_id-val ((m <ParamSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param_id-val is deprecated.  Use mavros-srv:param_id instead.")
  (param_id m))

(cl:ensure-generic-function 'integer-val :lambda-list '(m))
(cl:defmethod integer-val ((m <ParamSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:integer-val is deprecated.  Use mavros-srv:integer instead.")
  (integer m))

(cl:ensure-generic-function 'real-val :lambda-list '(m))
(cl:defmethod real-val ((m <ParamSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:real-val is deprecated.  Use mavros-srv:real instead.")
  (real m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamSet-request>) ostream)
  "Serializes a message object of type '<ParamSet-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'param_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'param_id))
  (cl:let* ((signed (cl:slot-value msg 'integer)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamSet-request>) istream)
  "Deserializes a message object of type '<ParamSet-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'param_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'integer) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'real) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamSet-request>)))
  "Returns string type for a service object of type '<ParamSet-request>"
  "mavros/ParamSetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamSet-request)))
  "Returns string type for a service object of type 'ParamSet-request"
  "mavros/ParamSetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamSet-request>)))
  "Returns md5sum for a message object of type '<ParamSet-request>"
  "67c3e64c93166b2201499faec7aea3ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamSet-request)))
  "Returns md5sum for a message object of type 'ParamSet-request"
  "67c3e64c93166b2201499faec7aea3ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamSet-request>)))
  "Returns full string definition for message of type '<ParamSet-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%string param_id~%int64 integer~%float64 real~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamSet-request)))
  "Returns full string definition for message of type 'ParamSet-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%string param_id~%int64 integer~%float64 real~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamSet-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'param_id))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamSet-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamSet-request
    (cl:cons ':param_id (param_id msg))
    (cl:cons ':integer (integer msg))
    (cl:cons ':real (real msg))
))
;//! \htmlinclude ParamSet-response.msg.html

(cl:defclass <ParamSet-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (integer
    :reader integer
    :initarg :integer
    :type cl:integer
    :initform 0)
   (real
    :reader real
    :initarg :real
    :type cl:float
    :initform 0.0))
)

(cl:defclass ParamSet-response (<ParamSet-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamSet-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamSet-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<ParamSet-response> is deprecated: use mavros-srv:ParamSet-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ParamSet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:success-val is deprecated.  Use mavros-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'integer-val :lambda-list '(m))
(cl:defmethod integer-val ((m <ParamSet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:integer-val is deprecated.  Use mavros-srv:integer instead.")
  (integer m))

(cl:ensure-generic-function 'real-val :lambda-list '(m))
(cl:defmethod real-val ((m <ParamSet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:real-val is deprecated.  Use mavros-srv:real instead.")
  (real m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamSet-response>) ostream)
  "Serializes a message object of type '<ParamSet-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'integer)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamSet-response>) istream)
  "Deserializes a message object of type '<ParamSet-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'integer) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'real) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamSet-response>)))
  "Returns string type for a service object of type '<ParamSet-response>"
  "mavros/ParamSetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamSet-response)))
  "Returns string type for a service object of type 'ParamSet-response"
  "mavros/ParamSetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamSet-response>)))
  "Returns md5sum for a message object of type '<ParamSet-response>"
  "67c3e64c93166b2201499faec7aea3ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamSet-response)))
  "Returns md5sum for a message object of type 'ParamSet-response"
  "67c3e64c93166b2201499faec7aea3ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamSet-response>)))
  "Returns full string definition for message of type '<ParamSet-response>"
  (cl:format cl:nil "bool success~%int64 integer~%float64 real~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamSet-response)))
  "Returns full string definition for message of type 'ParamSet-response"
  (cl:format cl:nil "bool success~%int64 integer~%float64 real~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamSet-response>))
  (cl:+ 0
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamSet-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamSet-response
    (cl:cons ':success (success msg))
    (cl:cons ':integer (integer msg))
    (cl:cons ':real (real msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ParamSet)))
  'ParamSet-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ParamSet)))
  'ParamSet-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamSet)))
  "Returns string type for a service object of type '<ParamSet>"
  "mavros/ParamSet")