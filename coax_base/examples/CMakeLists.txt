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
# 

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)


ADD_EXECUTABLE(
	servo-simple
	servo-simple.cpp
	PID.cpp
)

TARGET_LINK_LIBRARIES( servo-simple sbcom ${COMMON_LIBS})
	
ADD_EXECUTABLE(
	monitor
	monitor.cpp
)

TARGET_LINK_LIBRARIES( monitor sbcom ${COMMON_LIBS})
	
INSTALL_TARGETS(/bin monitor)

ADD_EXECUTABLE(
	coaxlog
	coaxlog.cpp
)

TARGET_LINK_LIBRARIES( coaxlog sbcom ${COMMON_LIBS})
	
INSTALL_TARGETS(/bin coaxlog)

IF(COMPILING_FOR_OSX)
	# No joystick on OSX
ELSE(COMPILING_FOR_OSX)
	ADD_EXECUTABLE(
		coaxjoy
		coaxjoy.cpp
	)

	TARGET_LINK_LIBRARIES(	coaxjoy sbcom ${COMMON_LIBS})
	INSTALL_TARGETS(/bin coaxjoy)
ENDIF(COMPILING_FOR_OSX)

ADD_EXECUTABLE(
	coaxbtcfg
	coaxbtcfg.cpp
)

TARGET_LINK_LIBRARIES(	coaxbtcfg sbcom ${COMMON_LIBS})
	
INSTALL_TARGETS(/bin coaxbtcfg)

ADD_EXECUTABLE(
	coaxtune
	coaxtune.cpp
)

TARGET_LINK_LIBRARIES(	coaxtune sbcom ${COMMON_LIBS})
	
INSTALL_TARGETS(/bin coaxtune)

ADD_EXECUTABLE(
	setdebug
	setdebug.cpp
)

TARGET_LINK_LIBRARIES(	setdebug sbcom ${COMMON_LIBS})
	
INSTALL_TARGETS(/bin setdebug)

ADD_EXECUTABLE(
	rangestat
	rangestat.cpp
)

TARGET_LINK_LIBRARIES(	rangestat sbcom ${COMMON_LIBS})
	
INSTALL_TARGETS(/bin rangestat)

ADD_EXECUTABLE(
	coaxmini
	coaxmini.cpp
)

TARGET_LINK_LIBRARIES(	coaxmini sbcom ${COMMON_LIBS})
	
ADD_EXECUTABLE(
	coaxeye
	coaxeye.cpp
)

TARGET_LINK_LIBRARIES(	coaxeye sbcom ${COMMON_LIBS})
	
ADD_EXECUTABLE(
	coax-sysid
	coax-sysid.cpp
)

TARGET_LINK_LIBRARIES(	coax-sysid sbcom ${COMMON_LIBS})
	
ADD_EXECUTABLE(
	coaxcustom
	coaxcustom.cpp
)

TARGET_LINK_LIBRARIES(	coaxcustom sbcom ${COMMON_LIBS})
	
ADD_EXECUTABLE(
	coaxraw
	coaxraw.cpp
)

TARGET_LINK_LIBRARIES(	coaxraw sbcom ${COMMON_LIBS})
	
ADD_EXECUTABLE(
	coaxrepeat
	coaxrepeat.cpp
)

TARGET_LINK_LIBRARIES(	coaxrepeat sbcom ${COMMON_LIBS})
INSTALL_TARGETS(/bin coaxrepeat)
	
