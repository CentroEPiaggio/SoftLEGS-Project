#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# CMakeLists used to compile the main sources files of the project
#

set(ROBOTRAN_MAJOR_VERSION 0)
set(ROBOTRAN_MINOR_VERSION 1)
set(ROBOTRAN_PATCH_VERSION 0)
set(ROBOTRAN_VERSION ${ROBOTRAN_MAJOR_VERSION}.${ROBOTRAN_MINOR_VERSION}.${ROBOTRAN_PATCH_VERSION})

set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

# list all source files
list_source_files(SYMBOLIC_SOURCE_FILES_C "*.c" "../symbolicR")
list_source_files(PROJECT_SOURCE_FILES_C  "*.c" "src/project")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/Simbody")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/integrator")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/jni")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/mbs_load_xml")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/post_process")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/real_time")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/robotran")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/robotran_yarp_plugin")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/sdl")
list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/useful")
# list_source_files(GENERIC_SOURCE_FILES_C  "*.c" "src/generic/other")

list_source_files(MAIN_SOURCE_FILES_C  "*.c" "src/generic/main_files")
set(SOURCE_FILES_C ${SYMBOLIC_SOURCE_FILES_C} ${PROJECT_SOURCE_FILES_C} ${GENERIC_SOURCE_FILES_C} ${SYMBOLIC_SOURCE_FILES_C}MAIN_SOURCE_FILES_C )

list_source_files(SYMBOLIC_SOURCE_FILES_CPP "*.cpp" "../symbolicR")
list_source_files(PROJECT_SOURCE_FILES_CPP  "*.cpp" "src/project")
list_source_files(PROJECT_SOURCE_FILES_CC   "*.cc" "src/project")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/Simbody")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/integrator")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/jni")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/mbs_load_xml")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/post_process")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/real_time")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/robotran")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/robotran_yarp_plugin")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/sdl")
list_source_files(GENERIC_SOURCE_FILES_CPP  "*.cpp" "src/generic/useful")
list_source_files(MAIN_SOURCE_FILES_CPP     "*.cpp" "src/generic/main_files")
set(SOURCE_FILES_CPP ${SYMBOLIC_SOURCE_FILES_CPP} ${PROJECT_SOURCE_FILES_CPP} ${PROJECT_SOURCE_FILES_CC} ${GENERIC_SOURCE_FILES_CPP} ${MAIN_SOURCE_FILES_CPP})
set_source_files_properties(${SOURCE_FILES_CPP} PROPERTIES LANGUAGE CXX )


# list_source_files_3(SOURCE_FILES_C   "*.c"   "../symbolicR" "src/project" "src/generic")
# list_source_files_3(SOURCE_FILES_CPP "*.cpp" "../symbolicR" "src/project" "src/generic")
list_source_files_3(HEADER_FILES_H   "*.h"   "../symbolicR" "src/project" "src/generic")
list_source_files_3(HEADER_FILES_HPP "*.hpp" "../symbolicR" "src/project" "src/generic")
list_source_files_3(HEADER_FILES_HH  "*.hh" "../symbolicR" "src/project" "src/generic")

set(HEADER_FILES ${HEADER_FILES_H} ${HEADER_FILES_HPP} ${HEADER_FILES_HH})

# message(INFO "SOURCE_FILES_C is ${SOURCE_FILES_C}")
# message(INFO "SOURCE_FILES_CPP is ${SOURCE_FILES_CPP}")

# list all source files from the 'generic' folder
list_folder_files(SRC_GEN_INT "src/generic/integrator")
list_folder_files(SRC_GEN_JNI "src/generic/jni")
list_folder_files(SRC_GEN_MAIN "src/generic/main_files")
list_folder_files(SRC_GEN_XML "src/generic/mbs_load_xml")
list_folder_files(SRC_GEN_OTH "src/generic/other")
list_folder_files(SRC_GEN_RT "src/generic/real_time")
list_folder_files(SRC_GEN_ROB "src/generic/robotran")
list_folder_files(SRC_GEN_SDL "src/generic/sdl")
list_folder_files(SRC_GEN_SYM "src/generic/Simbody")
list_folder_files(SRC_GEN_YARP "src/generic/robotran_yarp_plugin")

# list all source files from the 'project' folder
list_folder_files(SRC_PRO_CTRL "src/project/controller_files")
list_folder_files(SRC_PRO_PRO "src/project/project_files")
list_folder_files(SRC_PRO_SIMU "src/project/simulation_files")
list_folder_files(SRC_PRO_USR "src/project/user_files")
list_folder_files(SRC_PRO_SYM "../symbolicR")
list_special_files(SPECIAL_FILES)

# directories for the 'generic' folder
source_group("src\\generic\\integrator" FILES ${SRC_GEN_INT})
source_group("src\\generic\\jni" FILES ${SRC_GEN_JNI})
source_group("src\\generic\\main_files" FILES ${SRC_GEN_MAIN})
source_group("src\\generic\\mbs_load_xml" FILES ${SRC_GEN_XML})
source_group("src\\generic\\other" FILES ${SRC_GEN_OTH})
source_group("src\\generic\\real_time" FILES ${SRC_GEN_RT})
source_group("src\\generic\\robotran" FILES ${SRC_GEN_ROB})
source_group("src\\generic\\sdl" FILES ${SRC_GEN_SDL})
source_group("src\\generic\\symbody" FILES ${SRC_GEN_SYM})
source_group("src\\generic\\robotran_yarp_plugin" FILES ${SRC_GEN_YARP})

# directories for the 'project' folder
source_group("src\\project\\controller_files" FILES ${SRC_PRO_CTRL})
source_group("src\\project\\project_files" FILES ${SRC_PRO_PRO})
source_group("src\\project\\simulation_files" FILES ${SRC_PRO_SIMU})
source_group("src\\project\\user_files" FILES ${SRC_PRO_USR})
source_group("src\\project\\symbolicR" FILES ${SRC_PRO_SYM})
source_group("src\\project\\special_files" FILES ${SPECIAL_FILES}) 


# list all directories where the header files are located (except the external ones)
list_include_directories_2(FULL_INCLUDE_DIRECTORIES "src/generic" "src/project")

# include the corresponding directories
include_directories(${FULL_INCLUDE_DIRECTORIES})

set(SOURCE_FILES ${SOURCE_FILES_C})
set(SYMBOLIC_SOURCE_FILES ${SYMBOLIC_SOURCE_FILES_C})
set(PROJECT_SOURCE_FILES  ${PROJECT_SOURCE_FILES_C})
set(GENERIC_SOURCE_FILES  ${GENERIC_SOURCE_FILES_C})
set(MAIN_SOURCE_FILES     ${MAIN_SOURCE_FILES_C})
add_library(symbolic_lib_c SHARED ${SYMBOLIC_SOURCE_FILES_C} ${HEADER_FILES} ${SPECIAL_FILES})


if (FLAG_CXX_PROJECT)
	set(SOURCE_FILES ${SOURCE_FILES} ${SOURCE_FILES_CPP})
	set(SYMBOLIC_SOURCE_FILES ${SYMBOLIC_SOURCE_FILES} ${SYMBOLIC_SOURCE_FILES_CPP})
	set(PROJECT_SOURCE_FILES  ${PROJECT_SOURCE_FILES}  ${PROJECT_SOURCE_FILES_CPP})
	set(PROJECT_SOURCE_FILES  ${PROJECT_SOURCE_FILES}  ${PROJECT_SOURCE_FILES_CC})
	set(GENERIC_SOURCE_FILES  ${GENERIC_SOURCE_FILES}  ${GENERIC_SOURCE_FILES_CPP})
	set(MAIN_SOURCE_FILES     ${MAIN_SOURCE_FILES}     ${MAIN_SOURCE_FILES_CPP})
endif (FLAG_CXX_PROJECT)

# name of the executable: 'Exec'
set (Executable Exec)

# put the executable in the root of the build folder
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR} )

# set(BUILD_SHARED_LIBS ON)  //will it work for all libraries?


# create a library with all the simulation stuff inside, execpt for the main.
# This will be used by plugins as a reference to link against


add_library(project_lib  SHARED ${PROJECT_SOURCE_FILES}  ${HEADER_FILES} ${SPECIAL_FILES})
add_library(generic_lib  SHARED ${GENERIC_SOURCE_FILES}  ${HEADER_FILES} ${SPECIAL_FILES})
# add_library(generic_exept_main            ${SRC_GEN_INT} ${SRC_GEN_JNI} ${SRC_GEN_XML} ${SRC_GEN_OTH} ${SRC_GEN_RT} ${SRC_GEN_ROB} ${SRC_GEN_SDL} ${SRC_GEN_SYM} ${SRC_GEN_YARP})
set(robotran_libraries symbolic_lib_c project_lib generic_lib)

if(SYMBOLIC_SOURCE_FILES_CPP)
    add_library(symbolic_lib_cpp SHARED ${SYMBOLIC_SOURCE_FILES_CPP} ${HEADER_FILES} ${SPECIAL_FILES})
    set(robotran_libraries ${robotran_libraries} symbolic_lib_cpp)
endif()



# generate the executable
add_executable (${Executable} ${MAIN_SOURCE_FILES})
target_link_libraries (${Executable} ${robotran_libraries} ${robotran_libraries} ${robotran_libraries})

# math external library (for Unix): linkage
if (UNIX)
	target_link_libraries (${Executable} m)
        target_link_libraries (${Executable} pthread)
endif (UNIX)

# Libxml2 external libray: linkage
target_link_libraries (${Executable} ${LIBXML2_LIBRARIES})

# GSL external libray: linkage
target_link_libraries (${Executable} ${GSL_LIBRARIES})

if (FLAG_REAL_TIME)

	# SDL external library: linkage
	if (FLAG_SDL_INTERACTIONS)
		target_link_libraries (${Executable} ${SDL2_LIBRARIES})
		target_link_libraries (${Executable} ${SDL2TTF_LIBRARIES})
	endif (FLAG_SDL_INTERACTIONS)

	# JNI external library: linkage
	if (FLAG_JAVA_VISUALIZATION)
		target_link_libraries (${Executable} ${JNI_LIBRARIES})
	endif (FLAG_JAVA_VISUALIZATION)

endif (FLAG_REAL_TIME)

# Simbody
if (FLAG_SIMBODY)
	target_link_libraries (${Executable} ${SIMBODY_LIBRARIES})
endif(FLAG_SIMBODY)

# YARP
if (FLAG_YARP)

	SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

    	option(CREATE_SHARED_LIBRARY "Compile shared libraries rather than linking statically" TRUE)
	if (WIN32)
		set(YARP_FILTER_API TRUE)
	endif (WIN32)

	if (CREATE_SHARED_LIBRARY)
		set(BUILD_SHARED_LIBS ON)
		set(YARP_DLL ON)
	endif(CREATE_SHARED_LIBRARY)

	#TARGET_LINK_LIBRARIES(${Executable} ${YARP_LIBRARIES} ${ACE_LIBRARIES} curl) 
    	#TARGET_LINK_LIBRARIES(${Executable} ${YARP_LIBRARIES} ${ACE_LIBRARIES})

#     	message("ROBOTRAN_PLUGINS_SRC is ${ROBOTRAN_PLUGINS_SRC}")
    	add_subdirectory(generic/robotran_yarp_plugin/robotran_plugins)
#     	set(SOURCE_FILES_CPP ${SOURCE_FILES_CPP} ${ROBOTRAN_PLUGINS_SRC})
#     	set(SOURCE_FILES_C   ${SOURCE_FILES_C}   ${ROBOTRAN_PLUGINS_SRC})

    	target_link_libraries (${Executable} ${YARP_LIBRARIES} ${ACE_LIBRARIES} ) #robotranMotionControlPlugin robotranForceTorquePlugin)

endif (FLAG_YARP)


install(TARGETS ${robotran_libraries}
  # IMPORTANT: Add the foo library to the "export-set"
  EXPORT RobotranTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
#   PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/foo"
    COMPONENT dev)

install(TARGETS ${Executable}
  # IMPORTANT: Add the bar executable to the "export-set"
  EXPORT RobotranTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)


# files compiled as C++ files
# if (FLAG_CXX_PROJECT)
# 	set_source_files_properties(${SOURCE_FILES_CPP} PROPERTIES LANGUAGE CXX )
# endif (FLAG_CXX_PROJECT)


# CMake magic for exporting libraries
# ===============================

if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/Robotran)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Add all targets to the build-tree export set
export(TARGETS ${robotran_libraries} FILE "${PROJECT_BINARY_DIR}/RobotranTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE RoboTran)

# Create the RobotranConfig.cmake and RobotranConfigVersion files
# file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")

# ... for the build tree
foreach(folder ${FULL_INCLUDE_DIRECTORIES})
    set(ROBOTRAN_INCLUDE_DIRS "${ROBOTRAN_INCLUDE_DIRS}" "${CMAKE_SOURCE_DIR}/src/${folder}")
endforeach()
# set(ROBOTRAN_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(RobotranConfig.cmake.in "${PROJECT_BINARY_DIR}/ROBOTRANConfig.cmake" @ONLY)

# ... for the install tree
# set(CONF_INCLUDE_DIRS "\${INSTALL_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# configure_file(RobotranConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RobotranConfig.cmake" @ONLY)
# ... for both
configure_file(RobotranConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/ROBOTRANConfigVersion.cmake" @ONLY)

# Install the RobotranConfig.cmake and RobotranConfigVersion.cmake
install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ROBOTRANConfig.cmake" "${PROJECT_BINARY_DIR}/ROBOTRANConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}"
        COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT RobotranTargets
        DESTINATION "${INSTALL_CMAKE_DIR}"
        COMPONENT dev)

