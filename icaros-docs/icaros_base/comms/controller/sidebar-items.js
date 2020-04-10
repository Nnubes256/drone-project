initSidebarItems({"constant":[["_IMPL_DESERIALIZE_FOR_A2RMessage",""],["_IMPL_DESERIALIZE_FOR_R2ADesiredRotationAndThrottle",""],["_IMPL_DESERIALIZE_FOR_R2AMessage",""],["_IMPL_SERIALIZE_FOR_A2RMessage",""],["_IMPL_SERIALIZE_FOR_R2ADesiredRotationAndThrottle",""],["_IMPL_SERIALIZE_FOR_R2AMessage",""]],"enum":[["ControllerReceiveError","Error ocurred when receving packets from the flight controller"],["ControllerSendError","Error ocurred when sending packets to the flight controller"]],"fn":[["get_controller_codec","Obtains the `bincode2` configuration used for the flight controller's serialization/deserialization codec"]],"struct":[["A2RMessage","A flight-controller-to-ICAROS (Arduino-to-Raspberry) message."],["R2ADesiredRotationAndThrottle","The drone rotation and throttle inputted to the flight controller as part of the setpoint"],["R2AMessage","A ICAROS-to-flight-controller (Raspberry-to-Arduino) message."]],"trait":[["ControllerCommunicationService","A flight controller communication driver"]]});