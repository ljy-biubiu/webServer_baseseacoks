macro(add_app NAME)
    add_seasocks_executable(${NAME} ${NAME}.cpp)
endmacro()

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} include )
#add_app(ws_test)
add_seasocks_executable(ws_test ws_test.cpp getlidarc16.cpp)
target_link_libraries(ws_test PUBLIC jsoncpp ${PCL_LIBRARIES})

# target_include_directories(ws_test PUBLIC
#     jsoncpp
# )

add_app(ph_test)
add_app(serve)
add_app(ws_chatroom)
add_app(ws_echo)


if(NOT WIN32)
    add_app(ws_test_poll)
endif()
add_app(async_test)
target_link_libraries(async_test PRIVATE Threads::Threads)
add_app(multiserver)
target_link_libraries(multiserver PRIVATE Threads::Threads)
add_app(streaming_test)

function(copy_directory_post_build TARGET SUBDIRECTORY)
    add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${PROJECT_SOURCE_DIR}/${SUBDIRECTORY}" "$<TARGET_FILE_DIR:${TARGET}>/${SUBDIRECTORY}"
            )
endfunction()

copy_directory_post_build(ws_test src/ws_test_web)
copy_directory_post_build(async_test src/async_test_web)
copy_directory_post_build(ws_chatroom src/ws_chatroom_web)