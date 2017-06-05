set(ODE_DIRECTORIY ${ODE_DIRECTORIY} CACHE PATH "set ODE_DIRECTORIY ")

if(MSVC)
else(MSVC)
	if (CMAKE_VERSION VERSION_LESS "3.1")
	    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	      set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
	    endif ()
	else ()
	    set (CMAKE_CXX_STANDARD 11)
	endif ()
endif(MSVC)



if(NOT ODE_DIRECTORIY)
	message(FATAL_ERROR "Please set ODE_DIRECTORIY.")
endif()
set(ODEDir ${ODE_DIRECTORIY})







set(comp_srcs RaspberryPiMouseSimulator.cpp DrawThread_RasPiMouse.cpp RasPiMouseSimulatorObj.cpp searchFile.cpp  )
set(standalone_srcs RaspberryPiMouseSimulatorComp.cpp)


if(MSVC)
	set(drawstuff_resources ${ODEDir}/drawstuff/src/resources.rc)
	set(SHlWAPI shlwapi)
	set(ODELIB optimized;ode;debug;oded;optimized;drawstuff;debug;drawstuffd;;user32.lib;winmm;opengl32;glu32)

else(MSVC)
	set(ODELIB optimized;ode;debug;oded;optimized;drawstuff;debug;drawstuffd;;GL;GLU;X11)

endif(MSVC)



if (DEFINED OPENRTM_INCLUDE_DIRS)
  string(REGEX REPLACE "-I" ";"
    OPENRTM_INCLUDE_DIRS "${OPENRTM_INCLUDE_DIRS}")
  string(REGEX REPLACE " ;" ";"
    OPENRTM_INCLUDE_DIRS "${OPENRTM_INCLUDE_DIRS}")
endif (DEFINED OPENRTM_INCLUDE_DIRS)

if (DEFINED OPENRTM_LIBRARY_DIRS)
  string(REGEX REPLACE "-L" ";"
    OPENRTM_LIBRARY_DIRS "${OPENRTM_LIBRARY_DIRS}")
  string(REGEX REPLACE " ;" ";"
    OPENRTM_LIBRARY_DIRS "${OPENRTM_LIBRARY_DIRS}")
endif (DEFINED OPENRTM_LIBRARY_DIRS)

if (DEFINED OPENRTM_LIBRARIES)
  string(REGEX REPLACE "-l" ";"
    OPENRTM_LIBRARIES "${OPENRTM_LIBRARIES}")
  string(REGEX REPLACE " ;" ";"
    OPENRTM_LIBRARIES "${OPENRTM_LIBRARIES}")
endif (DEFINED OPENRTM_LIBRARIES)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${PROJECT_BINARY_DIR}/idl)
include_directories(${OPENRTM_INCLUDE_DIRS})
include_directories(${OMNIORB_INCLUDE_DIRS})
add_definitions(${OPENRTM_CFLAGS})
add_definitions(${OMNIORB_CFLAGS})


include_directories(${ODEDir}/include)

MAP_ADD_STR(comp_hdrs "../" comp_headers)

link_directories(${OPENRTM_LIBRARY_DIRS})
link_directories(${OMNIORB_LIBRARY_DIRS})


link_directories(${ODEDir}/lib/Release)
link_directories(${ODEDir}/lib/Debug)
if(MSVC)
	add_definitions(-DdDOUBLE -DCCD_DOUBLE)
else(MSVC)
	add_definitions(-DdDOUBLE -DCCD_DOUBLE -fPIC)
endif(MSVC)

add_library(${PROJECT_NAME} ${LIB_TYPE} ${comp_srcs}
  ${comp_headers} ${ALL_IDL_SRCS} ${drawstuff_resources})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
set_source_files_properties(${ALL_IDL_SRCS} PROPERTIES GENERATED 1)
if(NOT TARGET ALL_IDL_TGT)
 add_custom_target(ALL_IDL_TGT)
endif(NOT TARGET ALL_IDL_TGT)
add_dependencies(${PROJECT_NAME} ALL_IDL_TGT)
target_link_libraries(${PROJECT_NAME} ${OPENRTM_LIBRARIES} ${ODELIB} ${SHlWAPI})

add_executable(${PROJECT_NAME}Comp ${standalone_srcs}
  ${comp_srcs} ${comp_headers} ${ALL_IDL_SRCS} ${drawstuff_resources})
add_dependencies(${PROJECT_NAME}Comp ALL_IDL_TGT)
target_link_libraries(${PROJECT_NAME}Comp ${OPENRTM_LIBRARIES} ${ODELIB} ${SHlWAPI})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}Comp
    EXPORT ${PROJECT_NAME}
    RUNTIME DESTINATION ${INSTALL_PREFIX} COMPONENT component
    LIBRARY DESTINATION ${INSTALL_PREFIX} COMPONENT component
    ARCHIVE DESTINATION ${INSTALL_PREFIX} COMPONENT component)

install(FILES ${PROJECT_SOURCE_DIR}/RTC.xml DESTINATION ${INSTALL_PREFIX}
        COMPONENT component)
