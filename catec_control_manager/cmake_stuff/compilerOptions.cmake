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

SET(EXTRA_C_FLAGS                   "")
SET(EXTRA_C_FLAGS_RELEASE           "")   # Already contain "-O3 -DNDEBUG"
SET(EXTRA_C_FLAGS_DEBUG             "")

SET(EXTRA_EXE_LINKER_FLAGS          "")
SET(EXTRA_EXE_LINKER_FLAGS_RELEASE  "")
SET(EXTRA_EXE_LINKER_FLAGS_DEBUG    "")

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "c++-analyzer")

   SET(CMAKE_CXX_STANDARD 17)
   SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -std=c++17")

   SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -Wall")

   IF(${WARNINGS_ANSI_ISO})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wcast-align")
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -pedantic")
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wextra")
   ENDIF()

   IF(${WARNINGS_ARE_ERRORS})
     SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Werror")
   ENDIF()

   IF(${WARNINGS_EFFCPP})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Weffc++")
   ENDIF()

   SET(EXTRA_C_FLAGS_DEBUG "${EXTRA_C_FLAGS_DEBUG} -O0 -DDEBUG -D_DEBUG")
ENDIF()


IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

    ### Check C++17 ###
    EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    IF(NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
        MESSAGE(FATAL_ERROR "${PROJECT_NAME} C++17 support requires g++ 4.7 or greater.")
    ENDIF()

    # Makes all your symbols hidden by default.
    # Not valid for clang because cmake doesn't generate proper export files
    # SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -fvisibility=hidden")

    # Necessary for Qt
    SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -Wno-long-long")

    IF(NOT ${WARNINGS_ANSI_ISO})
        SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wno-narrowing")
        SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wno-delete-non-virtual-dtor")
        SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wno-unnamed-type-template-args")
    ENDIF()

ENDIF()

### Add user supplied extra options (optimization, etc...)
# ==========================================================
SET(EXTRA_C_FLAGS                   "${EXTRA_C_FLAGS}"
   CACHE INTERNAL "Extra compiler options")
SET(EXTRA_C_FLAGS_RELEASE           "${EXTRA_C_FLAGS_RELEASE}"
   CACHE INTERNAL "Extra compiler options for Release build")
SET(EXTRA_C_FLAGS_DEBUG             "${EXTRA_C_FLAGS_DEBUG}"
   CACHE INTERNAL "Extra compiler options for Debug build")
SET(EXTRA_EXE_LINKER_FLAGS          "${EXTRA_EXE_LINKER_FLAGS}"
   CACHE INTERNAL "Extra linker flags")
SET(EXTRA_EXE_LINKER_FLAGS_RELEASE  "${EXTRA_EXE_LINKER_FLAGS_RELEASE}"
   CACHE INTERNAL "Extra linker flags for Release build")
SET(EXTRA_EXE_LINKER_FLAGS_DEBUG    "${EXTRA_EXE_LINKER_FLAGS_DEBUG}"
   CACHE INTERNAL "Extra linker flags for Debug build")

### Combine all "extra" options
SET(CMAKE_C_FLAGS             "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${EXTRA_C_FLAGS_RELEASE}")
SET(CMAKE_C_FLAGS_DEBUG       "${CMAKE_C_FLAGS_DEBUG} ${EXTRA_C_FLAGS_DEBUG}")

SET(CMAKE_CXX_FLAGS           "${CMAKE_CXX_FLAGS} ${EXTRA_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_C_FLAGS_RELEASE}")
SET(CMAKE_CXX_FLAGS_DEBUG     "${CMAKE_CXX_FLAGS_DEBUG} ${EXTRA_C_FLAGS_DEBUG}")

SET(CMAKE_EXE_LINKER_FLAGS          "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE  "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG    "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")