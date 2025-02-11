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

INCLUDE(GenerateExportHeader)

# ------------------------------------------------------
# Build type
# ------------------------------------------------------
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release;RelWithDebInfo" CACHE STRING "Configs" FORCE)

IF(DEFINED CMAKE_BUILD_TYPE)
   SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "Debug")
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug"   AND
   NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release" AND
   NOT ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo")
   MESSAGE(FATAL_ERROR "Only Release and Debug build types are allowed.")
ENDIF()

# ----------------------------------------------------------------------------
# PROJECT CONFIGURATION
# force some variables that could be defined in the command line to be written to cache
# ----------------------------------------------------------------------------
OPTION(WARNINGS_ARE_ERRORS "Treat warnings as errors" OFF)
OPTION(WARNINGS_ANSI_ISO "Issue all the mandatory diagnostics Listed in C standard" ON)
OPTION(WARNINGS_EFFCPP "Issue all the warnings listed in the book of Scot Meyers" OFF)

OPTION(BUILD_UTILS "Build applications using the different modules" ON)
OPTION(BUILD_TESTS "Google Tests"                                   ON)