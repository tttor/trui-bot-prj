; Auto-generated. Do not edit!


(cl:in-package mavros-srv)


;//! \htmlinclude WaypointGOTO-request.msg.html

(cl:defclass <WaypointGOTO-request> (roslisp-msg-protocol:ros-message)
  ((waypoint
    :reader waypoint
    :initarg :waypoint
    :type mavros-msg:Waypoint
    :initform (cl:make-instance 'mavros-msg:Waypoint)))
)

(cl:defclass WaypointGOTO-request (<WaypointGOTO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointGOTO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointGOTO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<WaypointGOTO-request> is deprecated: use mavros-srv:WaypointGOTO-request instead.")))

(cl:ensure-generic-function 'waypoint-val :lambda-list '(m))
(cl:defmethod waypoint-val ((m <WaypointGOTO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:waypoint-val is deprecated.  Use mavros-srv:waypoint instead.")
  (waypoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointGOTO-request>) ostream)
  "Serializes a message object of type '<WaypointGOTO-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointGOTO-request>) istream)
  "Deserializes a message object of type '<WaypointGOTO-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointGOTO-request>)))
  "Returns string type for a service object of type '<WaypointGOTO-request>"
  "mavros/WaypointGOTORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointGOTO-request)))
  "Returns string type for a service object of type 'WaypointGOTO-request"
  "mavros/WaypointGOTORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointGOTO-request>)))
  "Returns md5sum for a message object of type '<WaypointGOTO-request>"
  "c52932a48a9836c9e3888b9ed4669183")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointGOTO-request)))
  "Returns md5sum for a message object of type 'WaypointGOTO-request"
  "c52932a48a9836c9e3888b9ed4669183")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointGOTO-request>)))
  "Returns full string definition for message of type '<WaypointGOTO-request>"
  (cl:format cl:nil "~%~%~%~%mavros/Waypoint waypoint~%~%================================================================================~%MSG: mavros/Waypoint~%# Waypoint.msg~%#~%# ROS representation of MAVLink MISSION_ITEM~%# See mavlink documentation~%~%~%~%# see enum MAV_FRAME~%uint8 frame~%uint8 FRAME_GLOBAL = 0~%uint8 FRAME_LOCAL_NED = 1~%uint8 FRAME_MISSION = 2~%uint8 FRAME_GLOBAL_REL_ALT = 3~%uint8 FRAME_LOCAL_ENU = 4~%~%# see enum MAV_CMD~%uint16 command~%uint16 NAV_WAYPOINT = 16~%uint16 NAV_LOITER_UNLIM = 17~%uint16 NAV_LOITER_TURNS = 18~%uint16 NAV_LOITER_TIME = 19~%uint16 NAV_RETURN_TO_LAUNCH = 20~%uint16 NAV_LAND = 21~%uint16 NAV_TAKEOFF = 22~%# TODO: ROI mode~%~%bool is_current~%bool autocontinue~%# meaning of this params described in enum MAV_CMD~%float32 param1~%float32 param2~%float32 param3~%float32 param4~%float64 x_lat~%float64 y_long~%float64 z_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointGOTO-request)))
  "Returns full string definition for message of type 'WaypointGOTO-request"
  (cl:format cl:nil "~%~%~%~%mavros/Waypoint waypoint~%~%================================================================================~%MSG: mavros/Waypoint~%# Waypoint.msg~%#~%# ROS representation of MAVLink MISSION_ITEM~%# See mavlink documentation~%~%~%~%# see enum MAV_FRAME~%uint8 frame~%uint8 FRAME_GLOBAL = 0~%uint8 FRAME_LOCAL_NED = 1~%uint8 FRAME_MISSION = 2~%uint8 FRAME_GLOBAL_REL_ALT = 3~%uint8 FRAME_LOCAL_ENU = 4~%~%# see enum MAV_CMD~%uint16 command~%uint16 NAV_WAYPOINT = 16~%uint16 NAV_LOITER_UNLIM = 17~%uint16 NAV_LOITER_TURNS = 18~%uint16 NAV_LOITER_TIME = 19~%uint16 NAV_RETURN_TO_LAUNCH = 20~%uint16 NAV_LAND = 21~%uint16 NAV_TAKEOFF = 22~%# TODO: ROI mode~%~%bool is_current~%bool autocontinue~%# meaning of this params described in enum MAV_CMD~%float32 param1~%float32 param2~%float32 param3~%float32 param4~%float64 x_lat~%float64 y_long~%float64 z_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointGOTO-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointGOTO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointGOTO-request
    (cl:cons ':waypoint (waypoint msg))
))
;//! \htmlinclude WaypointGOTO-response.msg.html

(cl:defclass <WaypointGOTO-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WaypointGOTO-response (<WaypointGOTO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointGOTO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointGOTO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros-srv:<WaypointGOTO-response> is deprecated: use mavros-srv:WaypointGOTO-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <WaypointGOTO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros-srv:success-val is deprecated.  Use mavros-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointGOTO-response>) ostream)
  "Serializes a message object of type '<WaypointGOTO-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointGOTO-response>) istream)
  "Deserializes a message object of type '<WaypointGOTO-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointGOTO-response>)))
  "Returns string type for a service object of type '<WaypointGOTO-response>"
  "mavros/WaypointGOTOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointGOTO-response)))
  "Returns string type for a service object of type 'WaypointGOTO-response"
  "mavros/WaypointGOTOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointGOTO-response>)))
  "Returns md5sum for a message object of type '<WaypointGOTO-response>"
  "c52932a48a9836c9e3888b9ed4669183")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointGOTO-response)))
  "Returns md5sum for a message object of type 'WaypointGOTO-response"
  "c52932a48a9836c9e3888b9ed4669183")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointGOTO-response>)))
  "Returns full string definition for message of type '<WaypointGOTO-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointGOTO-response)))
  "Returns full string definition for message of type 'WaypointGOTO-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointGOTO-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointGOTO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointGOTO-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WaypointGOTO)))
  'WaypointGOTO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WaypointGOTO)))
  'WaypointGOTO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointGOTO)))
  "Returns string type for a service object of type '<WaypointGOTO>"
  "mavros/WaypointGOTO")