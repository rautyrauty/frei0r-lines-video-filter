set (TARGET "frei0r-lines-video-filter")
set (SOURCES
        filter.cpp
        FrameHandler.cpp
        FrameHandler.h
)

if (MSVC)
        set (SOURCES ${SOURCES} frei0r_1_2.def)
endif (MSVC)

add_library (${TARGET} MODULE ${SOURCES})

set (LIBDIR "${CMAKE_INSTALL_LIBDIR}/frei0r-1")
install (TARGETS ${TARGET} LIBRARY DESTINATION ${LIBDIR})
