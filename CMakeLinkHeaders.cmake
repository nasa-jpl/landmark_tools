function(add_public_headers)
file(RELATIVE_PATH relative_path ${CMAKE_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_command(
OUTPUT ${CMAKE_SOURCE_DIR}/include/${relative_path}
COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/include/${relative_path}
)

foreach(header ${ARGV})
file(RELATIVE_PATH
temp ${CMAKE_SOURCE_DIR}/include/${relative_path}
${CMAKE_CURRENT_SOURCE_DIR}/${header})
add_custom_command(
DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header}
OUTPUT ${CMAKE_SOURCE_DIR}/include/${relative_path}/${header}
COMMAND ln -sf
ARGS ${temp} ${CMAKE_SOURCE_DIR}/include/${relative_path}
)
endforeach(header)

string(REPLACE "/" "_" target_name ${relative_path})
set(target_name ${target_name}_link_public_headers)
add_custom_target(
${target_name}
DEPENDS ${ARGV}
${CMAKE_SOURCE_DIR}/include/${relative_path}
)
add_dependencies(link_public_headers ${target_name})
endfunction(add_public_headers)