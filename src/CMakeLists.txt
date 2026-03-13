cmake_minimum_required(VERSION 3.10)
project(marcos_server)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL armv7l)
   # Zig/Clang uses underscores in CPU names; GCC uses hyphens.
   if (CMAKE_C_COMPILER_ID MATCHES "Clang")
      set (ARM_CPU_NAME "cortex_a9")
   else()
      set (ARM_CPU_NAME "cortex-a9")
   endif()

   # compile in release mode on the ARM
   set (CMAKE_C_FLAGS "-static -O3 -mcpu=${ARM_CPU_NAME} -mtune=${ARM_CPU_NAME} -mfpu=neon -mfloat-abi=hard -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-static -O3 -mcpu=${ARM_CPU_NAME} -mtune=${ARM_CPU_NAME} -mfpu=neon -mfloat-abi=hard -Wall -std=c++17")
else()
   # compile in debug mode on the host PC for testing
   set (CMAKE_C_FLAGS "-O0 -g -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-O0 -g -Wall -std=c++17")
   # set (CMAKE_C_FLAGS "-static -O3 -Wall -std=c11")
   # set (CMAKE_CXX_FLAGS "-static -O3 -std=c++14")
endif()

set(SERVER_SRC marcos_server.cpp hardware.cpp iface.cpp mpack/mpack.c)
add_executable(marcos_server ${SERVER_SRC})
