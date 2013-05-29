

# Source files
set( MainWindow_SOURCES MainWindow.cpp qnode.cpp )
# Header files for MOC processing (Qt4 stuff - VERY IMPORTANT)
set( MainWindow_HEADERS MainWindow.h qnode.hpp )
# Set resource files
set( MainWindow_RESOURCES MainWindow.qrc )

# Apply MOC 
QT4_WRAP_CPP( MainWindow_HEADERS_MOC ${MainWindow_HEADERS} )
# Process resources
QT4_ADD_RESOURCES( MainWindow_RESOURCES_RCC ${MainWindow_RESOURCES} )

# Generate executable
rosbuild_add_library( rosQtInterface2 ${MainWindow_SOURCES} ${MainWindow_HEADERS_MOC} ${MainWindow_RESOURCES_RCC} )
target_link_libraries( rosQtInterface2 ${QT_LIBRARIES} )
#set_target_properties( MainWindow PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )
