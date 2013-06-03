# **************************
# visualDataListener CMake file
# **************************

# Gather the .hpp files and run MOC throught them
# (if they have QT_OBJECT in the class, if not it does
# not matter)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

#Set the source files
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

# Create the executable
# How come we don't use rosbuild_add_executable? Mystery
# FYI, I just tried rosbuild_add_executable and that also works
add_executable( VisualDataListener ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries( VisualDataListener rosQtInterface2 )
