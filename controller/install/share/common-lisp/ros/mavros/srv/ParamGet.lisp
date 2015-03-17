; Auto-generated. Do not edit!


(cl:in-package mavros-srv)


;//! \htmlinclude ParamGet-request.msg.html

(cl:defclass <ParamGet-request> (roslisp-msg-protocol:ros-message)
  ((param_id
    :reader param_id
    :initarg :param_id
    :type cl:string
    :initform ""))
)

(cl:defclass ParamGet-request (<ParamGet-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamGet-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamGet-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<ParamGet-request> is deprecated: use mavros-srv:ParamGet-request instead.")))

(cl:ensure-generic-function 'param_id-val :lambda-list '(m))
(cl:defmethod param_id-val ((m <ParamGet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param_id-val is deprecated.  Use mavros-srv:param_id instead.")
  (param_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamGet-request>) ostream)
  "Serializes a message object of type '<ParamGet-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'param_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'param_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamGet-request>) istream)
  "Deserializes a message object of type '<ParamGet-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'param_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamGet-request>)))
  "Returns string type for a service object of type '<ParamGet-request>"
  "mavros/ParamGetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamGet-request)))
  "Returns string type for a service object of type 'ParamGet-request"
  "mavros/ParamGetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamGet-request>)))
  "Returns md5sum for a message object of type '<ParamGet-request>"
  "93fd6e4fa4f0fb943fff9ce5ce2182b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamGet-request)))
  "Returns md5sum for a message object of type 'ParamGet-request"
  "93fd6e4fa4f0fb943fff9ce5ce2182b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamGet-request>)))
  "Returns full string definition for message of type '<ParamGet-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%string param_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamGet-request)))
  "Returns full string definition for message of type 'ParamGet-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%string param_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamGet-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'param_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamGet-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamGet-request
    (cl:cons ':param_id (param_id msg))
))
;//! \htmlinclude ParamGet-response.msg.html

(cl:defclass <ParamGet-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ParamGet-response (<ParamGet-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamGet-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamGet-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<ParamGet-response> is deprecated: use mavros-srv:ParamGet-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ParamGet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:success-val is deprecated.  Use mavros-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'integer-val :lambda-list '(m))
(cl:defmethod integer-val ((m <ParamGet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:integer-val is deprecated.  Use mavros-srv:integer instead.")
  (integer m))

(cl:ensure-generic-function 'real-val :lambda-list '(m))
(cl:defmethod real-val ((m <ParamGet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:real-val is deprecated.  Use mavros-srv:real instead.")
  (real m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamGet-response>) ostream)
  "Serializes a message object of type '<ParamGet-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamGet-response>) istream)
  "Deserializes a message object of type '<ParamGet-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamGet-response>)))
  "Returns string type for a service object of type '<ParamGet-response>"
  "mavros/ParamGetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamGet-response)))
  "Returns string type for a service object of type 'ParamGet-response"
  "mavros/ParamGetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamGet-response>)))
  "Returns md5sum for a message object of type '<ParamGet-response>"
  "93fd6e4fa4f0fb943fff9ce5ce2182b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamGet-response)))
  "Returns md5sum for a message object of type 'ParamGet-response"
  "93fd6e4fa4f0fb943fff9ce5ce2182b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamGet-response>)))
  "Returns full string definition for message of type '<ParamGet-response>"
  (cl:format cl:nil "bool success~%int64 integer~%float64 real~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamGet-response)))
  "Returns full string definition for message of type 'ParamGet-response"
  (cl:format cl:nil "bool success~%int64 integer~%float64 real~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamGet-response>))
  (cl:+ 0
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamGet-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamGet-response
    (cl:cons ':success (success msg))
    (cl:cons ':integer (integer msg))
    (cl:cons ':real (real msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ParamGet)))
  'ParamGet-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ParamGet)))
  'ParamGet-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamGet)))
  "Returns string type for a service object of type '<ParamGet>"
  "mavros/ParamGet")