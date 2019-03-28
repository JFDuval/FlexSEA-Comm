file(GLOB FLEX_COMM_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

# Add the source to the parent project to create a overall library/DLL
set(FLEX_COMM_SRCS
	${FLEX_COMM_SRCS}
	PARENT_SCOPE
)