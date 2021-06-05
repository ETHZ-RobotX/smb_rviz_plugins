cmake_minimum_required(VERSION 3.0.2)
project(smb_rviz_plugins)
find_package(catkin REQUIRED COMPONENTS 
  rviz 
  sensor_msgs 
  std_msgs
  message_generation
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)
link_directories( ${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
  src/smb_battery_motor/smb_batt_mot_display.cpp
  src/smb_battery_payload/smb_batt_pay_display.cpp
  src/smb_battery_payload/smb_batt_panel.cpp
  src/smb_control/smb_control_panel.cpp
)

set(HDR_FILES
  include/smb_battery_motor/smb_batt_mot_display.hpp
  include/smb_battery_payload/smb_batt_pay_display.hpp
  include/smb_battery_payload/smb_batt_panel.hpp
  include/smb_control/smb_control_panel.hpp
)

add_library(
  ${PROJECT_NAME} 
  ${SRC_FILES} 
  ${HDR_FILES}
  resources/resources.qrc
)

# Messages
add_message_files(
  FILES
  SMBPower.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)