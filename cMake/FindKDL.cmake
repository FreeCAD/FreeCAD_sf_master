# Locate KDL install directory

# This module defines
# KDL_INSTALL where to find include, lib, bin, etc.
# KDL_FOUND, is set to true


IF ( CMAKE_PKGCONFIG_EXECUTABLE )

    MESSAGE( STATUS "Detecting KDL" )
    
    SET(ENV{PKG_CONFIG_PATH} "${KDL_INSTALL}/lib/pkgconfig/")
    MESSAGE( "Looking for KDL in: ${KDL_INSTALL}")
    PKGCONFIG( "orocos-kdl >= 0.99" KDL_FOUND KDL_INCLUDE_DIRS KDL_DEFINES KDL_LINK_DIRS KDL_LIBS )

    IF( KDL_FOUND )
        MESSAGE("   Includes in: ${KDL_INCLUDE_DIRS}")
        MESSAGE("   Libraries in: ${KDL_LINK_DIRS}")
        MESSAGE("   Libraries: ${KDL_LIBS}")
        MESSAGE("   Defines: ${KDL_DEFINES}")

	INCLUDE_DIRECTORIES( ${KDL_INCLUDE_DIRS} )
	LINK_DIRECTORIES( ${KDL_LINK_DIRS})


	SET(ENV{PKG_CONFIG_PATH} "${KDL_INSTALL}/lib/pkgconfig/:${OROCOS_INSTALL}/lib/pkgconfig")
	MESSAGE( "Looking for KDL Toolkit in: ${PKG_CONFIG_PATH}")
	PKGCONFIG( "orocos-kdltk-${OROCOS_TARGET} >= 0.99" KDLTK_FOUND KDLTK_INCLUDE_DIRS KDLTK_DEFINES KDLTK_LINK_DIRS KDLTK_LIBS )
	IF(KDLTK_FOUND)
	  INCLUDE_DIRECTORIES( ${KDLTK_INCLUDE_DIRS} )
	  LINK_DIRECTORIES( ${KDLTK_LINK_DIRS})
	  OROCOS_PKGCONFIG_INCPATH("${KDLTK_INCLUDE_DIRS}")
	  OROCOS_PKGCONFIG_LIBPATH("${KDLTK_LINK_DIRS}")
	  OROCOS_PKGCONFIG_LIBS("${KDLTK_LIBS}")
	  IF(CORBA_ENABLED)
	    SET(ENV{PKG_CONFIG_PATH} "${KDL_INSTALL}/lib/pkgconfig/:${OROCOS_INSTALL}/lib/pkgconfig")
	    MESSAGE("Looking for KDL Toolkit CORBA extension in ${PKG_CONFIG_PATH}")
	    PKGCONFIG( "orocos-kdltk-corba-${OROCOS_TARGET} >= 0.99" KDLTKCORBA_FOUND KDLTKCORBA_INCLUDE_DIRS KDLTKCORBA_DEFINES KDLTKCORBA_LINK_DIRS KDLTKCORBA_LIBS )
	    IF(KDLTKCORBA_FOUND)
	      INCLUDE_DIRECTORIES( ${KDLTKCORBA_INCLUDE_DIRS} )
	      LINK_DIRECTORIES( ${KDLTKCORBA_LINK_DIRS})
	      OROCOS_PKGCONFIG_INCPATH("${KDLTKCORBA_INCLUDE_DIRS}")
	      OROCOS_PKGCONFIG_LIBPATH("${KDLTKCORBA_LINK_DIRS}")
	      OROCOS_PKGCONFIG_LIBS("${KDLTKCORBA_LIBS}")
	    ENDIF ( KDLTKCORBA_FOUND )
	  ENDIF(CORBA_ENABLED)
	ENDIF ( KDLTK_FOUND )
      ENDIF ( KDL_FOUND )

ELSE  ( CMAKE_PKGCONFIG_EXECUTABLE )

    # Can't find pkg-config -- have to search manually
    MESSAGE( FATAL_ERROR "Can't find KDL without pkgconfig !")

ENDIF ( CMAKE_PKGCONFIG_EXECUTABLE )
