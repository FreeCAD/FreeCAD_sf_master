OPTION(FREECAD_USE_EXTERNAL_SMESH "Use system installed smesh instead of the bundled." ON)

set(FREETYPE_INCLUDE_DIRS ${FREECAD_LIBPACK_DIR}/include/freetype2)
set(FREETYPE_LIBRARY optimized ${FREECAD_LIBPACK_DIR}/lib/freetype.lib debug ${FREECAD_LIBPACK_DIR}/lib/freetyped.lib)

SET(SHIBOKEN_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include/shiboken2)
SET(SHIBOKEN_LIBRARY     optimized ${FREECAD_LIBPACK_DIR}/lib/shiboken2.cp37-win_amd64.lib debug ${FREECAD_LIBPACK_DIR}/lib/shiboken2_d_d.cp37-win_amd64.lib)
#set(SHIBOKEN_BINARY      ${FREECAD_LIBPACK_DIR}/bin/shiboken)

SET(PYSIDE_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include/PySide2)
SET(PYSIDE_LIBRARY     optimized ${FREECAD_LIBPACK_DIR}/lib/pyside2.cp37-win_amd64.lib debug ${FREECAD_LIBPACK_DIR}/lib/pyside2_d.cp37-win_amd64_d.lib)
SET(PYSIDE_PYTHONPATH  ${FREECAD_LIBPACK_DIR}/pyside/Lib/site-packages)
SET(PYSIDE_TYPESYSTEMS ${FREECAD_LIBPACK_DIR}/pyside/share/PySide/typesystems)

set(EIGEN3_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include/eigen3)

set(Boost_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include CACHE PATH "" FORCE)
SET(Boost_USE_DEBUG_PYTHON TRUE)

set(OCC_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include/opencascade )

set(OCE_DIR ${FREECAD_LIBPACK_DIR}/lib/cmake CACHE PATH "" FORCE)

set(SWIG_EXECUTABLE ${FREECAD_LIBPACK_DIR}/bin/swig/swig.exe CACHE FILEPATH "Swig" FORCE)

set(PYTHON_EXECUTABLE ${FREECAD_LIBPACK_DIR}/bin/python.exe CACHE FILEPATH "" FORCE)
set(PYTHON_LIBRARY ${FREECAD_LIBPACK_DIR}/lib/python37.lib CACHE FILEPATH "" FORCE)
set(PYTHON_DEBUG_LIBRARY ${FREECAD_LIBPACK_DIR}/lib/python37_d.lib CACHE FILEPATH "" FORCE)
set(PYTHON_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include/python3.7 CACHE PATH "" FORCE)

if (BUILD_QT5)
  # default Qt5 stuff
  # Set paths to cmake config files for each Qt module
  set(Qt5_ROOT_DIR ${FREECAD_LIBPACK_DIR} CACHE PATH "")

  set (Qt5_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5 CACHE PATH "")
  set (Qt5AxBase_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5AxBase CACHE PATH "")
  set (Qt5AxContainer_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5AxContainer CACHE PATH "")
  set (Qt5AxServer_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5AxServer CACHE PATH "")
  set (Qt5Concurrent_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Concurrent CACHE PATH "")
  set (Qt5Core_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Core CACHE PATH "")
  set (Qt5DBus_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5DBus CACHE PATH "")
  set (Qt5Designer_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Designer CACHE PATH "")
  set (Qt5Gui_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Gui CACHE PATH "")
  set (Qt5Help_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Help CACHE PATH "")
  set (Qt5LinguistTools_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5LinguistTools CACHE PATH "")
  set (Qt5Multimedia_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Multimedia CACHE PATH "")
  set (Qt5MultimediaWidgets_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5MultimediaWidgets CACHE PATH "")
  set (Qt5Network_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Network CACHE PATH "")
  set (Qt5OpenGL_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5OpenGL CACHE PATH "")
  set (Qt5OpenGLExtensions_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5OpenGLExtensions CACHE PATH "")
  set (Qt5PrintSupport_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5PrintSupport CACHE PATH "")
  set (Qt5Qml_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Qml CACHE PATH "")
  set (Qt5Quick_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Quick CACHE PATH "")
  set (Qt5QuickTest_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5QuickTest CACHE PATH "")
  set (Qt5QuickWidgets_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5QuickWidgets CACHE PATH "")
  set (Qt5Sql_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Sql CACHE PATH "")
  set (Qt5Svg_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Svg CACHE PATH "")
  set (Qt5Test_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Test CACHE PATH "")
  set (Qt5UiPlugin_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5UiPlugin CACHE PATH "")
  set (Qt5UiTools_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5UiTools CACHE PATH "")
  set (Qt5Widgets_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Widgets CACHE PATH "")
  set (Qt5Xml_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5Xml CACHE PATH "")
  set (Qt5XmlPatterns_DIR ${Qt5_ROOT_DIR}/lib/cmake/Qt5XmlPatterns CACHE PATH "")
endif (BUILD_QT5)

SET(XercesC_INCLUDE_DIRS "${FREECAD_LIBPACK_DIR}/include/xercesc")
SET(XercesC_LIBRARIES optimized ${FREECAD_LIBPACK_DIR}/lib/xerces-c_3.lib debug ${FREECAD_LIBPACK_DIR}/lib/xerces-c_3D.lib)
SET(XercesC_FOUND "YES" )

find_library(COIN3D_LIBRARY_RELEASE coin4 "${FREECAD_LIBPACK_DIR}/lib")
find_library(COIN3D_LIBRARY_DEBUG coin4d "${FREECAD_LIBPACK_DIR}/lib")
set(COIN3D_LIBRARIES optimized ${COIN3D_LIBRARY_RELEASE}
                     debug ${COIN3D_LIBRARY_DEBUG})
SET(COIN3D_INCLUDE_DIRS ${FREECAD_LIBPACK_DIR}/include)
set(COIN3D_FOUND TRUE)

set(NETGENDATA ${FREECAD_LIBPACK_DIR}/include/netgen)

if(FREECAD_USE_FREETYPE)
    set(FREETYPE_INCLUDE_DIR_freetype2 ${FREECAD_LIBPACK_DIR}/include/freetype2)
endif(FREECAD_USE_FREETYPE)

set(ZLIB_INCLUDE_DIR ${FREECAD_LIBPACK_DIR}/include)
find_library(ZLIB_LIBRARY zlib.lib "${FREECAD_LIBPACK_DIR}/lib")
find_library(ZLIB_DEBUG_ZLIB_LIBRARY zlibd.lib "${FREECAD_LIBPACK_DIR}/lib")

link_directories(${FREECAD_LIBPACK_DIR}/lib)
set(PCL_INCLUDE_DIRS ${FREECAD_LIBPACK_DIR}/include/pcl-1.9)
set(PCL_LIBRARY_DIRS ${FREECAD_LIBPACK_DIR}/lib)

set(PCL_COMMON_LIBRARIES optimized pcl_common_release debug pcl_common_debug)
set(PCL_FEATURES_LIBRARIES optimized pcl_features_release debug pcl_features_debug)
set(PCL_FILTERS_LIBRARIES optimized pcl_filters_release debug pcl_filters_debug)
set(PCL_IO_LIBRARIES optimized pcl_io_release debug pcl_io_debug)
set(PCL_IO_PLY_LIBRARIES optimized pcl_io_ply_release debug pcl_io_ply_debug)
set(PCL_KDTREE_LIBRARIES optimized pcl_kdtree_release debug pcl_kdtree_debug)
set(PCL_KEYPOINTS_LIBRARIES optimized pcl_keypoints_release debug pcl_keypoints_debug)
set(PCL_ML_LIBRARIES optimized pcl_ml_release debug pcl_ml_debug)
set(PCL_OCTREE_LIBRARIES optimized pcl_octree_release debug pcl_octree_debug)
set(PCL_RECOGNITION_LIBRARIES optimized pcl_recognition_release debug pcl_recognition_debug)
set(PCL_REGISTRATION_LIBRARIES optimized pcl_registration_release debug pcl_registration_debug)
set(PCL_SAMPLE_CONSENSUS_LIBRARIES optimized pcl_sample_consensus_release debug pcl_sample_consensus_debug)
set(PCL_SEARCH_LIBRARIES optimized pcl_search_release debug pcl_search_debug)
set(PCL_SEGMENTATION_LIBRARIES optimized pcl_segmentation_release debug pcl_segmentation_debug)
set(PCL_STEREO_LIBRARIES optimized pcl_stereo_release debug pcl_stereo_debug)
set(PCL_SURFACE_LIBRARIES optimized pcl_surface_release debug pcl_surface_debug)
set(PCL_TRACKING_LIBRARIES optimized pcl_tracking_release debug pcl_tracking_debug)

set(PCL_LIBRARIES
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FEATURES_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_IO_PLY_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    ${PCL_KEYPOINTS_LIBRARIES}
    ${PCL_ML_LIBRARIES}
    ${PCL_OCTREE_LIBRARIES}
    ${PCL_RECOGNITION_LIBRARIES}
    ${PCL_REGISTRATION_LIBRARIES}
    ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
    ${PCL_SEARCH_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_STEREO_LIBRARIES}
    ${PCL_SURFACE_LIBRARIES}
    ${PCL_TRACKING_LIBRARIES}
)
set(PCL_FOUND TRUE)
set(PCL_COMMON_FOUND TRUE)
set(PCL_FEATURES_FOUND TRUE)
set(PCL_FILTERS_FOUND TRUE)
set(PCL_IO_FOUND TRUE)
set(PCL_IO_PLY_FOUND TRUE)
set(PCL_KDTREE_FOUND TRUE)
set(PCL_KEYPOINTS_FOUND TRUE)
set(PCL_ML_FOUND TRUE)
set(PCL_OCTREE_FOUND TRUE)
set(PCL_RECOGNITION_FOUND TRUE)
set(PCL_REGISTRATION_FOUND TRUE)
set(PCL_SAMPLE_CONSENSUS_FOUND TRUE)
set(PCL_SEARCH_FOUND TRUE)
set(PCL_SEGMENTATION_FOUND TRUE)
set(PCL_STEREO_FOUND TRUE)
set(PCL_SURFACE_FOUND TRUE)
set(PCL_TRACKING_FOUND TRUE)

SET(VTK_DIR ${FREECAD_LIBPACK_DIR}/lib/cmake/vtk-8.2/)


macro(fc_wrap_cpp outfiles )
	QT4_EXTRACT_OPTIONS(moc_files moc_options ${ARGN})
	# fixes bug 0000585: bug with boost 1.48
	SET(moc_options ${moc_options} -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
	SET(ARGN)
	foreach(it ${moc_files})
		get_filename_component(it ${it} ABSOLUTE)
		QT4_MAKE_OUTPUT_FILE(${it} moc_ cpp outfile)
		ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
			COMMAND ${QT_MOC_EXECUTABLE}
			ARGS ${moc_options} ${it} -o ${outfile}
			MAIN_DEPENDENCY ${it}
		)
		SET(${outfiles} ${${outfiles}} ${outfile})
		add_file_dependencies(${it} ${outfile})
	endforeach(it)
endmacro(fc_wrap_cpp)

MACRO (QT4_EXTRACT_OPTIONS _qt4_files _qt4_options)
	SET(${_qt4_files})
	SET(${_qt4_options})
	#SET(_QT4_DOING_OPTIONS FALSE)
	FOREACH(_currentArg ${ARGN})
	#  IF ("${_currentArg}" STREQUAL "OPTIONS")
	#	SET(_QT4_DOING_OPTIONS TRUE)
	#  ELSE ("${_currentArg}" STREQUAL "OPTIONS")
	#	IF(_QT4_DOING_OPTIONS) 
	#	  LIST(APPEND ${_qt4_options} "${_currentArg}")
	#	ELSE(_QT4_DOING_OPTIONS)
		  LIST(APPEND ${_qt4_files} "${_currentArg}")
	#	ENDIF(_QT4_DOING_OPTIONS)
	#  ENDIF ("${_currentArg}" STREQUAL "OPTIONS")
	ENDFOREACH(_currentArg)  
ENDMACRO (QT4_EXTRACT_OPTIONS)

# macro used to create the names of output files preserving relative dirs
MACRO (QT4_MAKE_OUTPUT_FILE infile prefix ext outfile )
  STRING(LENGTH ${CMAKE_CURRENT_BINARY_DIR} _binlength)
  STRING(LENGTH ${infile} _infileLength)
  SET(_checkinfile ${CMAKE_CURRENT_SOURCE_DIR})
  IF(_infileLength GREATER _binlength)
    STRING(SUBSTRING "${infile}" 0 ${_binlength} _checkinfile)
    IF(_checkinfile STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
      FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_BINARY_DIR} ${infile})
    ELSE(_checkinfile STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
      FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${infile})
    ENDIF(_checkinfile STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  ELSE(_infileLength GREATER _binlength)
    FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${infile})
  ENDIF(_infileLength GREATER _binlength)
  SET(_outfile "${CMAKE_CURRENT_BINARY_DIR}/${rel}")
  STRING(REPLACE ".." "__" _outfile ${_outfile})
  GET_FILENAME_COMPONENT(outpath ${_outfile} PATH)
  GET_FILENAME_COMPONENT(_outfile ${_outfile} NAME_WE)
  FILE(MAKE_DIRECTORY ${outpath})
  SET(${outfile} ${outpath}/${prefix}${_outfile}.${ext})
ENDMACRO (QT4_MAKE_OUTPUT_FILE )