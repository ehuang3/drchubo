set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

# Find Qt4
find_package( Qt4 4.4.3 REQUIRED QTCore QtGui QtXml )
include( ${QT_USE_FILE} ) 
add_definitions( ${QT_DEFINITIONS} )

# Find OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories( ${OpenCV_LIB_DIR} )

# Source files
set( driver_SOURCES driverInterface.cpp )
# Header files for MOC processing (Qt4 stuff - VERY IMPORTANT)
set( driver_HEADERS cvImageWidget.h )

# Apply MOC 
QT4_WRAP_CPP( driver_HEADERS_MOC ${driver_HEADERS} )

# Generate executable
rosbuild_add_executable( driverInterface ${driver_SOURCES} ${driver_HEADERS_MOC} )
target_link_libraries( driverInterface ${QT_LIBRARIES} opencv_core opencv_imgproc opencv_highgui )
set_target_properties( driverInterface PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )


