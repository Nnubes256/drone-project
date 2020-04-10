initSidebarItems({"constant":[["HEADER_VALUE","Constant value of the header byte"],["_IMPL_DESERIALIZE_FOR_A2GCommandType",""],["_IMPL_DESERIALIZE_FOR_A2GMessage",""],["_IMPL_DESERIALIZE_FOR_CommandTypeInternal",""],["_IMPL_DESERIALIZE_FOR_G2ACommandType",""],["_IMPL_DESERIALIZE_FOR_G2AControllerAxisState",""],["_IMPL_DESERIALIZE_FOR_G2AHeartbeat",""],["_IMPL_DESERIALIZE_FOR_G2AMessage",""],["_IMPL_DESERIALIZE_FOR_GPSNoData",""],["_IMPL_SERIALIZE_FOR_A2GCommandType",""],["_IMPL_SERIALIZE_FOR_A2GMessage",""],["_IMPL_SERIALIZE_FOR_CommandTypeInternal",""],["_IMPL_SERIALIZE_FOR_G2ACommandType",""],["_IMPL_SERIALIZE_FOR_G2AControllerAxisState",""],["_IMPL_SERIALIZE_FOR_G2AHeartbeat",""],["_IMPL_SERIALIZE_FOR_G2AMessage",""],["_IMPL_SERIALIZE_FOR_GPSNoData",""]],"enum":[["A2GCommandType","Defines the possible command types of an air-to-ground command"],["G2ACommandType","Defines the possible command types of a ground-to-air command"],["ReceiveError","Error ocurred when receving packets from ground control"],["SendError","Error ocurred when sending packets to ground control"]],"fn":[["get_air_codec","Obtains the `bincode2` configuration used for the ground control's serialization/deserialization codec"]],"mod":[["scheduler","Packet scheduler for air-to-ground and corresponding trait (interface) implementations"]],"struct":[["A2GMessage","Defines the full structure of a air-to-ground packet"],["CommandTypeInternal","Middle-layer internal structure involved in command serialization/deserialization."],["G2AControllerAxisState","The gamepad input given to the drone from ground control"],["G2AHeartbeat","UNUSED"],["G2AMessage","Defines the full structure of a ground-to-air packet"],["GPSNoData","A zero-sized type representing the lack of GPS data."]],"trait":[["AirCommunicationService","An air-to-ground communication driver"]]});