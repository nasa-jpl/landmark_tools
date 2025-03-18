function(add_public_headers)

if(NOT DEFINED landmark_tools_SOURCE_DIR)
    set(landmark_tools_SOURCE_DIR ${CMAKE_SOURCE_DIR})
endif()

file(RELATIVE_PATH relative_path ${landmark_tools_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "relative_path ${relative_path}")

add_custom_command(
OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/include/${relative_path}
COMMAND mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/include/${relative_path}
)

foreach(header ${ARGV})
file(RELATIVE_PATH
temp ${CMAKE_CURRENT_SOURCE_DIR}/include/${relative_path}
${CMAKE_CURRENT_SOURCE_DIR}/${header})
add_custom_command(
DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header}
OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/include/${relative_path}/${header}
COMMAND ln -sf
ARGS ${temp} ${CMAKE_CURRENT_SOURCE_DIR}/include/${relative_path}
)
endforeach(header)

string(REPLACE "/" "_" target_name ${relative_path})
set(target_name ${target_name}_link_public_headers)
message(STATUS "target_name ${target_name}")
add_custom_target(
${target_name}
DEPENDS ${ARGV}
${CMAKE_CURRENT_SOURCE_DIR}/include/${relative_path}
)
add_dependencies(link_public_headers ${target_name})
endfunction(add_public_headers)