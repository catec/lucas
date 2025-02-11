##---------------------------------------------------------------------------------------------------------------------
##  LUCAS: Lightweight framework for UAV Control And Supervision
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
##---------------------------------------------------------------------------------------------------------------------
## This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
## License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
## version.
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
## warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
## You should have received a copy of the GNU General Public License along with this program. If not, see
## https://www.gnu.org/licenses/.
##---------------------------------------------------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# Display status message for important variables
# ----------------------------------------------------------------------------
MESSAGE(STATUS)
MESSAGE(STATUS "------------- General configuration for ${PROJECT_NAME} - ${VERSION} -------------")
MESSAGE(STATUS)
MESSAGE(STATUS "Generator:    ${CMAKE_GENERATOR}")
MESSAGE(STATUS "Compiler:     ${CMAKE_CXX_COMPILER_ID}")

IF(${CMAKE_BUILD_TYPE} STREQUAL "Release")
   MESSAGE(STATUS "C flags (Release):     ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
   MESSAGE(STATUS "C++ flags (Release):   ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
ELSE()
   MESSAGE(STATUS "C flags (Debug):       ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG}")
   MESSAGE(STATUS "C++ flags (Debug):     ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

MESSAGE(STATUS)
MESSAGE(STATUS "------------------------------------------------------------------")
MESSAGE(STATUS)
MESSAGE(STATUS)
MESSAGE(STATUS "CMAKE_BUILD_TYPE       = ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR = ${CMAKE_SYSTEM_PROCESSOR}")
MESSAGE(STATUS "CMAKE_INSTALL_PREFIX   = ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS)
MESSAGE(STATUS "WARNINGS_ANSI_ISO      = ${WARNINGS_ANSI_ISO}")
MESSAGE(STATUS "WARNINGS_ARE_ERRORS    = ${WARNINGS_ARE_ERRORS}")
MESSAGE(STATUS "WARNINGS_EFFCPP        = ${WARNINGS_EFFCPP}")
MESSAGE(STATUS)
MESSAGE(STATUS "BUILD_UTILS            = ${BUILD_UTILS}")
MESSAGE(STATUS)
