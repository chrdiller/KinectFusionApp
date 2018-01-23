###############################################################################
# Find OpenNI2
#
# This sets the following variables:
# OPENNI2_FOUND - True if OPENNI was found.
# OPENNI2_INCLUDE_DIRS - Directories containing the OPENNI include files.
# OPENNI2_LIBRARIES - Libraries needed to use OPENNI.

find_package(PkgConfig)
if (${CMAKE_VERSION} VERSION_LESS 2.8.2)
    pkg_check_modules(PC_OPENNI openni2-dev)
else ()
    pkg_check_modules(PC_OPENNI QUIET openni2-dev)
endif ()

set(OPENNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

#add a hint so that it can find it without the pkg-config
find_path(OPENNI2_INCLUDE_DIR OpenNI.h
        HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} ${OPENNI2_INCLUDE} /usr/include/openni2 /usr/include/ni2 /usr/local/include/ni2
        PATHS "$ENV{PROGRAMFILES}/OpenNI2/Include" "$ENV{PROGRAMW6432}/OpenNI2/Include"
        PATH_SUFFIXES openni ni)
#add a hint so that it can find it without the pkg-config
find_library(OPENNI2_LIBRARY
        NAMES OpenNI2
        HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS} ${OPENNI2_REDIST} /usr/lib /usr/local/lib/ni2
        PATHS "$ENV{PROGRAMFILES}/OpenNI2/Redist" "$ENV{PROGRAMW6432}/OpenNI2/Redist" "$ENV{PROGRAMW6432}/OpenNI2"
        PATH_SUFFIXES lib lib64
        )

set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
if (APPLE)
    set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})
else ()
    set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG
        OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)

mark_as_advanced(OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)
if (OPENNI2_FOUND)
    include_directories(${OPENNI2_INCLUDE_DIRS})
    message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIR}, lib: ${OPENNI2_LIBRARY})")
endif (OPENNI2_FOUND)