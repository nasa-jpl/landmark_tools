function(add_public_headers)
    file(RELATIVE_PATH relative_path ${landmark_tools_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})

    set(include_dir ${landmark_tools_SOURCE_DIR}/include/${relative_path})
    
    # Ensure the include directory is explicitly created
    file(MAKE_DIRECTORY ${include_dir})

    # Debug: Print which directories are being created
    message(STATUS "Creating include directory: ${include_dir}")

    foreach(header ${ARGV})
        set(source ${CMAKE_CURRENT_SOURCE_DIR}/${header})
        set(target ${include_dir}/${header})
        message(STATUS "Linking ${source} to ${target}")
        add_custom_command(
            DEPENDS ${source}
            OUTPUT ${target}
            COMMAND ln -sf ${source} ${target}
            COMMENT "Symlinking ${source} to ${target}"
        )
    endforeach(header)

    string(REPLACE "/" "_" target_name ${relative_path})
    if(NOT target_name)
        set(target_name "root")
    endif()
    set(target_name ${CMAKE_CURRENT_SOURCE_DIR}_${target_name}_link_public_headers)
    string(REPLACE "/" "_" target_name ${target_name})  # Make it CMake-safe

    add_custom_target(
        ${target_name}
        DEPENDS ${ARGV} ${include_dir}
    )

    add_dependencies(link_public_headers ${target_name})
endfunction()
