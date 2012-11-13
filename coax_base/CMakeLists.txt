# 
# All rights reserved.
# 
# Skybotix API is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# Skybotix API is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.


CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6)
MARK_AS_ADVANCED( CMAKE_BACKWARDS_COMPATIBILITY )

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Define project
# PROJECT( coax-communication )
MESSAGE( STATUS "Configuring project ${PROJECT_NAME}" )

# lib-version 0.1.0
SET( ${PROJECT_NAME}_MAJOR_VERSION 0 )
SET( ${PROJECT_NAME}_MINOR_VERSION 1 )
SET( ${PROJECT_NAME}_PATCH_LEVEL   0 )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


FIND_LIBRARY(SYSLIBS pthread rt m)
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/src)


IF( USE_READLINE)
	SET( READLINE_DEF -DUSE_READLINE )
	SET( READLINE_LIB -lreadline )
ENDIF( USE_READLINE)

INCLUDE_DIRECTORIES(${CMAKE_HOME_DIRECTORY}/include)


SET(COMPILING_FOR_OSX no)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
SET(COMPILING_FOR_OSX yes)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

ADD_DEFINITIONS(-ggdb -Wall -D_GNU_SOURCE -DDEFENSIVE)
ADD_DEFINITIONS(-DSBC_HAS_IO -DSBC_HAS_COMM -DSBC_HAS_PTHREAD)
ADD_DEFINITIONS(-DLINUX ${READLINE_DEF} -DSIMULATION)
#ADD_DEFINITIONS(-DUSE_SOCKET)
#ADD_DEFINITIONS( -ggdb -march=pentium4 -O3 -pipe -Wall )
SET(COMMON_LIBS pthread rt m)

IF (COMPILING_FOR_OSX)
	ADD_DEFINITIONS(-DMACOSX)
	SET(COMMON_LIBS pthread m) 
ENDIF (COMPILING_FOR_OSX)

SUBDIRS(
	src 
  #	examples
	repeater
	helishell
	# bootloader
)
	

