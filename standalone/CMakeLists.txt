cmake_minimum_required(VERSION 3.0.2)
project(turntable_standalone)

FIND_LIBRARY(GPIB_LIBRARY adgpib64 /usr/lib/)

include_directories(
  ../usbgpib/include
)

add_executable(turntable turntable.cpp Turntable.hpp)

target_link_libraries(turntable
  ${GPIB_LIBRARY}
)

install(TARGETS turntable
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
