#####################
# Sections
#####################

 message( status " Current bynary dir: ${CMAKE_CURRENT_BINARY_DIR}" )
set(QT_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../resources/images.qrc)
set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/../ui/main_window.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp)

QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})



######################
# Sources
######################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

#####################
# Binaries
#####################

rosbuild_add_library( rosQtInterface ${QT_SOURCES} ${QT_MOC_HPP} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} )
target_link_libraries( rosQtInterface ${QT_LIBRARIES})
