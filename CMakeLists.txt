project (FlexSEA_comm )

set(COMM_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_buffers.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_circular_buffer.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_comm.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_comm_multi.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_interface.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_payload.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/flexsea_multi_circbuff.c
)

# Point to all the place sto find header files
include_directories (
	inc
	flexsea-comm/inc
	flexsea-projects/inc
	flexsea-projects/ActPack/inc
	flexsea-projects/Rigid/inc
	flexsea-system/inc
)

# Add the source to the parent project to create a overall library/DLL
set( COMM_LIB_SRCS 
	${COMM_SRCS}
	PARENT_SCOPE
)

add_definitions(-DBUILD_SHARED_LIB_DLL -DTEST_PC -DINCLUDE_UPROJ_ACTPACK -DBOARD_TYPE_FLEXSEA_PLAN)

add_library( FlexSEA_comm  STATIC ${COMM_SRCS} )
set_property(TARGET FlexSEA_comm  PROPERTY POSITION_INDEPENDENT_CODE ON)