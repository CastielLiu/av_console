##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.0)

project(av_console)

add_compile_options(-std=c++11)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    diagnostic_msgs
    sensor_msgs
    driverless
    driverless_common
    actionlib
    actionlib_msgs
    ant_msgs
)
include_directories(${catkin_INCLUDE_DIRS} include/ driverless)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(av_console ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(av_console ${av_console_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(av_console ${QT_LIBRARIES} ${catkin_LIBRARIES} /usr/local/lib/libtinyxml2.a)
install(TARGETS av_console RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#install av_console automatically
#execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/release/install.sh" OUTPUT_VARIABLE cmd_res)
#message(++++++++++++++++++++ "${cmd_res}")


