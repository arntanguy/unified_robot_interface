### CMake macro
macro(add_protocol NAME)
  file(
    GLOB_RECURSE
    PROTOCOL_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/robot_comm/${NAME}/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/robot_comm/${NAME}/*.h
  )

  add_library(${NAME} SHARED)
  add_library(${PROJECT_NAME}::${NAME} ALIAS ${NAME})

  target_link_libraries(${NAME} PUBLIC robot_comm_interface)

  target_sources(${NAME} PRIVATE ${PROTOCOL_SOURCES})

  target_include_directories(
    ${NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
                   $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(robot_comm PUBLIC ${NAME})

  install(
    TARGETS ${NAME}
    EXPORT ${NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endmacro()

### Serialization
macro(add_serializer NAME)
  add_library(${NAME} SHARED ${ARGN})

  target_include_directories(
    ${NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
                   $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(robot_comm_serializer INTERFACE ${NAME})
endmacro()

add_library(robot_comm_serializer INTERFACE)
add_library(
  ${PROJECT_NAME}::communication::serializer ALIAS robot_comm_serializer
)

add_serializer(
  robot_comm_flatbuffers serialization/FlatBufferSerializer.cpp
  ../include/robot_comm/serialization/FlatbufferSerializer.h
)

target_link_libraries(robot_comm_flatbuffers PUBLIC mc_rtc_flatbuffers)

add_dependencies(robot_comm_flatbuffers GENERATE_mc_rtc_flatbuffers)

if(TARGET proto-mc-rtc)
  add_serializer(
    robot_comm_protobuf serialization/ProtobufSerializer.cpp
    ../include/robot_comm/serialization/ProtobufSerializer.h
  )
  target_link_libraries(robot_comm_protobuf PUBLIC proto-mc-rtc)
endif()

###  Communication Interface
set(COMMUNICATION_INTERFACE_SRC Communication.cpp)

set(COMMUNICATION_INTERFACE_HDR ../include/robot_comm/Communication.h)

add_library(
  robot_comm_interface SHARED ${COMMUNICATION_INTERFACE_SRC}
                              ${COMMUNICATION_INTERFACE_HDR}
)
add_library(${PROJECT_NAME}::robot_comm_interface ALIAS robot_comm_interface)

target_link_libraries(
  robot_comm_interface PUBLIC mc_rtc::mc_rtc_utils robot_comm_serializer
)

if(TARGET robot_comm_protobuf)
  target_link_libraries(robot_comm_interface PRIVATE robot_comm_protobuf)
  target_compile_definitions(robot_comm_interface PRIVATE WITH_PROTOBUF)
endif()

target_include_directories(
  robot_comm_interface
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:include>
)

### Communication Factory

set(COMMUNICATION_FACTORY_SRC CommunicationFactory.cpp
                              ../include/robot_comm/CommunicationFactory.h
)

add_library(robot_comm SHARED ${COMMUNICATION_FACTORY_SRC})
add_library(${PROJECT_NAME}::robot_comm ALIAS robot_comm)

target_link_libraries(
  robot_comm PUBLIC mc_rtc::mc_rtc_utils robot_comm_serializer
)

target_include_directories(
  robot_comm PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
                    $<INSTALL_INTERFACE:include>
)

if(TARGET proto-mc-rtc)
  install(
    TARGETS robot_comm robot_comm_interface robot_comm_serializer
            robot_comm_protobuf robot_comm_flatbuffers
    EXPORT robot_commTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
else()
  install(
    TARGETS robot_comm robot_comm_interface robot_comm_serializer
            robot_comm_flatbuffers
    EXPORT robot_commTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif()

### Add Protocols

add_protocol(zenoh)
target_link_libraries(zenoh PUBLIC zenohcxx::zenohc)

# add_protocol(shm)
