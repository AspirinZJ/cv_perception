cmake_minimum_required(VERSION 3.0.2)
project(rtcm)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtcm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
    include ${catkin_INCLUDE_DIRS}
    include/
)


add_executable(str2str src/str2str.c
                    src/rtklib/rcv/binex.c
                    src/rtklib/stream.c 
                    src/rtklib/streamsvr.c 
                    src/rtklib/rtkcmn.c 
                    src/rtklib/solution.c 
                    src/rtklib/sbas.c 
                    src/rtklib/geoid.c 
                    src/rtklib/rcvraw.c 
                    src/rtklib/rcv/novatel.c 
                    src/rtklib/rcv/ublox.c
                    src/rtklib/rcv/ss2.c 
                    src/rtklib/rcv/crescent.c 
                    src/rtklib/rcv/skytraq.c 
                    src/rtklib/rcv/gw10.c 
                    src/rtklib/rcv/javad.c 
                    src/rtklib/rcv/nvs.c 
                    src/rtklib/rcv/rt17.c 
                    src/rtklib/rtcm.c 
                    src/rtklib/rtcm2.c 
                    src/rtklib/rtcm3.c 
                    src/rtklib/rtcm3e.c 
                    src/rtklib/preceph.c)

add_dependencies(str2str ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(str2str
    ${catkin_LIBRARIES} m pthread rt
)