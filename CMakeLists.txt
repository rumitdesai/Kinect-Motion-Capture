cmake_minimum_required(VERSION 2.8.11)

# default build type
if(NOT CMAKE_BUILD_TYPE)
     set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build" FORCE)
     set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

set(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE})
# Force VS GUI to only give (it forces to only give two configuration type "CMAKE_BUILD_TYPE")
mark_as_advanced(FORCE CMAKE_CONFIGURATION_TYPES)

#User can custom project names from CMake GUI
set(KMOTION_PROJECT_NAME "KMotion" CACHE STRING "KMotion Project Name")
project(${KMOTION_PROJECT_NAME})

# set the root directory variable
set(KMOTION_ROOTDIR ${PROJECT_SOURCE_DIR})
if(DEFINED ENV{KMOTION_THIRDPARTY})
      file(TO_CMAKE_PATH $ENV{KMOTION_THIRDPARTY} kmotion_thirdparty)
endif()
set(KMOTION_THIRDPARTY ${kmotion_thirdparty} CACHE STRING "Third party directory")

# Install path
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX "${KMOTION_THIRDPARTY}/KMotion" CACHE PATH "Install prefix" FORCE)
endif()

#Set Target platform variable
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(KMOTION_TARGET_WINDOWS 1)
endif()

# Enabling the property for using folders 
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Target platform
if(KMOTION_TARGET_WINDOWS)
	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		set(KMOTION_PLATFORM "win64d")
	else()
		set(KMOTION_PLATFORM "win64")
	endif()
endif()


set(KMOTION_BUILD_PREFIX ${CMAKE_BINARY_DIR})

# installation
set(KMOTION_INSTALL_BIN "bin/${KMOTION_PLATFORM}")


# shared installation / build location
if (CMAKE_INSTALL_PREFIX STREQUAL CMAKE_BINARY_DIR)
	set(CMAKE_BULD_WITH_INSTALL_RPATH TRUE)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${KMOTION_BUILD_PREFIX}/${KMOTION_INSTALL_BIN}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${KMOTION_BUILD_PREFIX}/${KMOTION_INSTALL_BIN}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${KMOTION_BUILD_PREFIX}/${KMOTION_INSTALL_BIN}")

foreach(c ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${c} c)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${c} "${KMOTION_BUILD_PREFIX}/${KMOTION_INSTALL_BIN}")
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${c} "${KMOTION_BUILD_PREFIX}/${KMOTION_INSTALL_BIN}")
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${c} "${KMOTION_BUILD_PREFIX}/${KMOTION_INSTALL_BIN}")
endforeach()


function(KMotion_SetGroups files)
	foreach(i IN LISTS files)
		get_filename_component(source_path "${i}" ABSOLUTE)
		file(RELATIVE_PATH source_path "${CMAKE_SOURCE_DIR}" "${source_path}")
		get_filename_component(source_path "${source_path}" PATH)
		string(REPLACE "/" "\\" source_path "${source_path}")

		source_group("${source_path}" FILES ${i})
	endforeach()
endfunction()


# Kinect library and include setup
set(kinect_root "${KMOTION_THIRDPARTY}/Kinect")
set(KINECT_INCLUDES "${kinect_root}/v2.0_1409/inc/")
set(KINECT_LIBS "${kinect_root}/v2.0_1409/Lib/x64/Kinect20.lib")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
	install(FILES "${kinect_root}/v2.0_1409/dll/Kinect20.dll" DESTINATION ${KMOTION_INSTALL_BIN})
endif()


# Qt library and include setup
set(qt_root "${KMOTION_THIRDPARTY}/Qt/5.4/msvc2013_64")
set(QT_INCLUDES "${qt_root}/include/")
set(Qt5Widgets_DIR "${qt_root}/lib/cmake/Qt5Widgets/")
set(QT_LIBS "${qt_root}/lib/Qt5Core.lib" "${qt_root}/lib/Qt5Gui.lib" "${qt_root}/lib/Qt5Widgets.lib")

install(FILES "${qt_root}/bin/Qt5Core.dll" DESTINATION ${KMOTION_INSTALL_BIN})
install(FILES "${qt_root}/bin/Qt5Gui.dll" DESTINATION ${KMOTION_INSTALL_BIN})
install(FILES "${qt_root}/bin/Qt5Widgets.dll" DESTINATION ${KMOTION_INSTALL_BIN})

install(FILES "${qt_root}/bin/icudt53.dll" DESTINATION ${KMOTION_INSTALL_BIN})
install(FILES "${qt_root}/bin/icuin53.dll" DESTINATION ${KMOTION_INSTALL_BIN})
install(FILES "${qt_root}/bin/icuuc53.dll" DESTINATION ${KMOTION_INSTALL_BIN})


# OpenGL library and include setup
set(opengl_root "${KMOTION_THIRDPARTY}/OpenGL/freeglut-MSVC-3.0.0-2.mp/freeglut")
set(OPENGL_INCLUDES "${opengl_root}/include/GL/")
set(OPENGL_LIBS "${opengl_root}/lib/x64/freeglut.lib" "${KMOTION_THIRDPARTY}/OpenGL/opengl32.lib")

# GLU include setup
set(GLU_INCLUDES "${KMOTION_THIRDPARTY}/GLU/")
set(GLU_LIBS "${GLU_INCLUDES}/glu32.lib")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	install(FILES "${KMOTION_THIRDPARTY}/GLU/glu32.dll" DESTINATION ${KMOTION_INSTALL_BIN})
	install(FILES "${KMOTION_THIRDPARTY}/OpenGL/opengl32.dll" DESTINATION ${KMOTION_INSTALL_BIN})
endif()


# eigen include setup
set(EIGEN_INCLUDES "${KMOTION_THIRDPARTY}/Eigen/")


add_subdirectory("KMotion")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	install(FILES "${KMOTION_BUILD_PREFIX}/bin/${KMOTION_PLATFORM}/${KMOTION_PROJECT_NAME}.pdb" DESTINATION ${KMOTION_INSTALL_BIN})
	install(FILES "${KMOTION_BUILD_PREFIX}/bin/${KMOTION_PLATFORM}/${KMOTION_PROJECT_NAME}.ilk" DESTINATION ${KMOTION_INSTALL_BIN})
endif()

install(FILES "${KMOTION_BUILD_PREFIX}/bin/${KMOTION_PLATFORM}/${KMOTION_PROJECT_NAME}.exe" DESTINATION ${KMOTION_INSTALL_BIN})