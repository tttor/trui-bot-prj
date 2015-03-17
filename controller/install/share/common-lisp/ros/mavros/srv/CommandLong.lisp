; Auto-generated. Do not edit!


(cl:in-package mavros-srv)


;//! \htmlinclude CommandLong-request.msg.html

(cl:defclass <CommandLong-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (confirmation
    :reader confirmation
    :initarg :confirmation
    :type cl:fixnum
    :initform 0)
   (param1
    :reader param1
    :initarg :param1
    :type cl:float
    :initform 0.0)
   (param2
    :reader param2
    :initarg :param2
    :type cl:float
    :initform 0.0)
   (param3
    :reader param3
    :initarg :param3
    :type cl:float
    :initform 0.0)
   (param4
    :reader param4
    :initarg :param4
    :type cl:float
    :initform 0.0)
   (param5
    :reader param5
    :initarg :param5
    :type cl:float
    :initform 0.0)
   (param6
    :reader param6
    :initarg :param6
    :type cl:float
    :initform 0.0)
   (param7
    :reader param7
    :initarg :param7
    :type cl:float
    :initform 0.0))
)

(cl:defclass CommandLong-request (<CommandLong-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandLong-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandLong-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<CommandLong-request> is deprecated: use mavros-srv:CommandLong-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:command-val is deprecated.  Use mavros-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'confirmation-val :lambda-list '(m))
(cl:defmethod confirmation-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:confirmation-val is deprecated.  Use mavros-srv:confirmation instead.")
  (confirmation m))

(cl:ensure-generic-function 'param1-val :lambda-list '(m))
(cl:defmethod param1-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param1-val is deprecated.  Use mavros-srv:param1 instead.")
  (param1 m))

(cl:ensure-generic-function 'param2-val :lambda-list '(m))
(cl:defmethod param2-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param2-val is deprecated.  Use mavros-srv:param2 instead.")
  (param2 m))

(cl:ensure-generic-function 'param3-val :lambda-list '(m))
(cl:defmethod param3-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param3-val is deprecated.  Use mavros-srv:param3 instead.")
  (param3 m))

(cl:ensure-generic-function 'param4-val :lambda-list '(m))
(cl:defmethod param4-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param4-val is deprecated.  Use mavros-srv:param4 instead.")
  (param4 m))

(cl:ensure-generic-function 'param5-val :lambda-list '(m))
(cl:defmethod param5-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param5-val is deprecated.  Use mavros-srv:param5 instead.")
  (param5 m))

(cl:ensure-generic-function 'param6-val :lambda-list '(m))
(cl:defmethod param6-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param6-val is deprecated.  Use mavros-srv:param6 instead.")
  (param6 m))

(cl:ensure-generic-function 'param7-val :lambda-list '(m))
(cl:defmethod param7-val ((m <CommandLong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:param7-val is deprecated.  Use mavros-srv:param7 instead.")
  (param7 m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CommandLong-request>)))
    "Constants for message type '<CommandLong-request>"
  '((:CMD_DO_SET_MODE . 176)
    (:CMD_DO_JUMP . 177)
    (:CMD_DO_CHANGE_SPEED . 178)
    (:CMD_DO_SET_HOME . 179)
    (:CMD_DO_SET_RELAY . 181)
    (:CMD_DO_REPEAT_RELAY . 182)
    (:CMD_DO_SET_SERVO . 183)
    (:CMD_DO_REPEAT_SERVO . 184)
    (:CMD_DO_CONTROL_VIDEO . 200)
    (:CMD_DO_SET_ROI . 201)
    (:CMD_DO_MOUNT_CONTROL . 205)
    (:CMD_DO_SET_CAM_TRIGG_DIST . 206)
    (:CMD_DO_FENCE_ENABLE . 207)
    (:CMD_DO_PARACHUTE . 208)
    (:CMD_DO_INVERTED_FLOGHT . 210)
    (:CMD_DO_MOUNT_CONTROL_QUAT . 220)
    (:CMD_PREFLIGHT_CALIBRATION . 241)
    (:CMD_MISSION_START . 300)
    (:CMD_COMPONENT_ARM_DISARM . 400)
    (:CMD_START_RX_PAIR . 500))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CommandLong-request)))
    "Constants for message type 'CommandLong-request"
  '((:CMD_DO_SET_MODE . 176)
    (:CMD_DO_JUMP . 177)
    (:CMD_DO_CHANGE_SPEED . 178)
    (:CMD_DO_SET_HOME . 179)
    (:CMD_DO_SET_RELAY . 181)
    (:CMD_DO_REPEAT_RELAY . 182)
    (:CMD_DO_SET_SERVO . 183)
    (:CMD_DO_REPEAT_SERVO . 184)
    (:CMD_DO_CONTROL_VIDEO . 200)
    (:CMD_DO_SET_ROI . 201)
    (:CMD_DO_MOUNT_CONTROL . 205)
    (:CMD_DO_SET_CAM_TRIGG_DIST . 206)
    (:CMD_DO_FENCE_ENABLE . 207)
    (:CMD_DO_PARACHUTE . 208)
    (:CMD_DO_INVERTED_FLOGHT . 210)
    (:CMD_DO_MOUNT_CONTROL_QUAT . 220)
    (:CMD_PREFLIGHT_CALIBRATION . 241)
    (:CMD_MISSION_START . 300)
    (:CMD_COMPONENT_ARM_DISARM . 400)
    (:CMD_START_RX_PAIR . 500))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandLong-request>) ostream)
  "Serializes a message object of type '<CommandLong-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confirmation)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param7))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandLong-request>) istream)
  "Deserializes a message object of type '<CommandLong-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confirmation)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param5) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param6) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param7) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandLong-request>)))
  "Returns string type for a service object of type '<CommandLong-request>"
  "mavros/CommandLongRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandLong-request)))
  "Returns string type for a service object of type 'CommandLong-request"
  "mavros/CommandLongRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandLong-request>)))
  "Returns md5sum for a message object of type '<CommandLong-request>"
  "61a4b6ac245fda8075fde260e82de332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandLong-request)))
  "Returns md5sum for a message object of type 'CommandLong-request"
  "61a4b6ac245fda8075fde260e82de332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandLong-request>)))
  "Returns full string definition for message of type '<CommandLong-request>"
  (cl:format cl:nil "~%~%~%uint16 CMD_DO_SET_MODE = 176~%uint16 CMD_DO_JUMP = 177~%uint16 CMD_DO_CHANGE_SPEED = 178~%uint16 CMD_DO_SET_HOME = 179~%uint16 CMD_DO_SET_RELAY = 181~%uint16 CMD_DO_REPEAT_RELAY = 182~%uint16 CMD_DO_SET_SERVO = 183~%uint16 CMD_DO_REPEAT_SERVO = 184~%uint16 CMD_DO_CONTROL_VIDEO = 200~%uint16 CMD_DO_SET_ROI = 201~%uint16 CMD_DO_MOUNT_CONTROL = 205~%uint16 CMD_DO_SET_CAM_TRIGG_DIST = 206~%uint16 CMD_DO_FENCE_ENABLE = 207~%uint16 CMD_DO_PARACHUTE = 208~%uint16 CMD_DO_INVERTED_FLOGHT = 210~%uint16 CMD_DO_MOUNT_CONTROL_QUAT = 220~%uint16 CMD_PREFLIGHT_CALIBRATION = 241~%uint16 CMD_MISSION_START = 300~%uint16 CMD_COMPONENT_ARM_DISARM = 400~%uint16 CMD_START_RX_PAIR = 500~%~%uint16 command~%uint8 confirmation~%float32 param1~%float32 param2~%float32 param3~%float32 param4~%float32 param5~%float32 param6~%float32 param7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandLong-request)))
  "Returns full string definition for message of type 'CommandLong-request"
  (cl:format cl:nil "~%~%~%uint16 CMD_DO_SET_MODE = 176~%uint16 CMD_DO_JUMP = 177~%uint16 CMD_DO_CHANGE_SPEED = 178~%uint16 CMD_DO_SET_HOME = 179~%uint16 CMD_DO_SET_RELAY = 181~%uint16 CMD_DO_REPEAT_RELAY = 182~%uint16 CMD_DO_SET_SERVO = 183~%uint16 CMD_DO_REPEAT_SERVO = 184~%uint16 CMD_DO_CONTROL_VIDEO = 200~%uint16 CMD_DO_SET_ROI = 201~%uint16 CMD_DO_MOUNT_CONTROL = 205~%uint16 CMD_DO_SET_CAM_TRIGG_DIST = 206~%uint16 CMD_DO_FENCE_ENABLE = 207~%uint16 CMD_DO_PARACHUTE = 208~%uint16 CMD_DO_INVERTED_FLOGHT = 210~%uint16 CMD_DO_MOUNT_CONTROL_QUAT = 220~%uint16 CMD_PREFLIGHT_CALIBRATION = 241~%uint16 CMD_MISSION_START = 300~%uint16 CMD_COMPONENT_ARM_DISARM = 400~%uint16 CMD_START_RX_PAIR = 500~%~%uint16 command~%uint8 confirmation~%float32 param1~%float32 param2~%float32 param3~%float32 param4~%float32 param5~%float32 param6~%float32 param7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandLong-request>))
  (cl:+ 0
     2
     1
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandLong-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandLong-request
    (cl:cons ':command (command msg))
    (cl:cons ':confirmation (confirmation msg))
    (cl:cons ':param1 (param1 msg))
    (cl:cons ':param2 (param2 msg))
    (cl:cons ':param3 (param3 msg))
    (cl:cons ':param4 (param4 msg))
    (cl:cons ':param5 (param5 msg))
    (cl:cons ':param6 (param6 msg))
    (cl:cons ':param7 (param7 msg))
))
;//! \htmlinclude CommandLong-response.msg.html

(cl:defclass <CommandLong-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CommandLong-response (<CommandLong-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandLong-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandLong-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<CommandLong-response> is deprecated: use mavros-srv:CommandLong-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CommandLong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:success-val is deprecated.  Use mavros-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <CommandLong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:result-val is deprecated.  Use mavros-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandLong-response>) ostream)
  "Serializes a message object of type '<CommandLong-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandLong-response>) istream)
  "Deserializes a message object of type '<CommandLong-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandLong-response>)))
  "Returns string type for a service object of type '<CommandLong-response>"
  "mavros/CommandLongResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandLong-response)))
  "Returns string type for a service object of type 'CommandLong-response"
  "mavros/CommandLongResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandLong-response>)))
  "Returns md5sum for a message object of type '<CommandLong-response>"
  "61a4b6ac245fda8075fde260e82de332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandLong-response)))
  "Returns md5sum for a message object of type 'CommandLong-response"
  "61a4b6ac245fda8075fde260e82de332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandLong-response>)))
  "Returns full string definition for message of type '<CommandLong-response>"
  (cl:format cl:nil "bool success~%~%uint8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandLong-response)))
  "Returns full string definition for message of type 'CommandLong-response"
  (cl:format cl:nil "bool success~%~%uint8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandLong-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandLong-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandLong-response
    (cl:cons ':success (success msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CommandLong)))
  'CommandLong-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CommandLong)))
  'CommandLong-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandLong)))
  "Returns string type for a service object of type '<CommandLong>"
  "mavros/CommandLong")