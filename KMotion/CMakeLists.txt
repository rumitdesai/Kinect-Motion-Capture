set(object
	Object/wrist_smallsize.dat)

set(sources
	colorDisplay.h
	colorDisplay.cpp
	colorthread.cpp
	ControlPosition.h
	ControlPosition.cpp
	glWidget.cpp
	JointSelectionDialog.cpp
	limbSelectionDlg.cpp
	main.cpp
	maindialog.cpp
	motion.cpp
	motion.h
	motionProperties.h
	motionProperties.cpp
	pugiconfig.hpp
	pugixml.hpp
	pugixml.cpp
	Sensor.cpp
	Skeletonthread.cpp
	UI_color.h
	UI_color.cpp
	)

set(math_lib
	Math/dualNumber.h
	Math/dualNumber.cpp
	Math/dualQuaternion.h
	Math/dualQuaternion.cpp
	Math/hMatrix.h
	Math/hMatrix.cpp
	Math/hPoint.h
	Math/hPoint.cpp
	Math/point2D.h
	Math/point2D.cpp
	Math/Quaternion.h
	Math/Quaternion.cpp
	)

set(moc_headers
	colorthread.h
	maindialog.h
	glWidget.h
	JointSelectionDialog.h
	limbSelectionDlg.h
	Skeletonthread.h
	Sensor.h
	)

set(UI
	maindialog.ui
	)

set(Resource
	maindialog.qrc
	)

set(includes
	${KINECT_INCLUDES}
	${QT_INCLUDES}
	${OPENGL_INCLUDES}
	${GLU_INCLUDES}
	${EIGEN_INCLUDES}
	)

set(libs
	${OPENGL_LIBS}
	${GLU_LIBS}
	${KINECT_LIBS}
	${QT_LIBS}
	)

set(disp_file
	${sources}
	${moc_headers}
	${math_lib}
	)



find_package(Qt5Widgets REQUIRED)
add_definitions(${QT_DEFINITIONS})
include_directories(${includes})
include_directories(${CMAKE_BINARY_DIR})


QT5_ADD_RESOURCES(RES_SOURCES ${Resource})
QT5_WRAP_UI(UI_HEADERS ${UI})
QT5_WRAP_CPP(MOC_SRC ${moc_headers})


add_executable(KMotion ${sources} ${math_lib} ${MOC_SRC} ${RES_SOURCES} ${UI_HEADERS})
target_link_libraries(KMotion ${libs})

set_target_properties(KMotion PROPERTIES OUTPUT_NAME "KMotion")

set_property(TARGET KMotion PROPERTY FOLDER "Framework")
KMotion_SetGroups("${disp_file}")
install(FILES "${object}" DESTINATION "${KMOTION_INSTALL_BIN}/Object/")