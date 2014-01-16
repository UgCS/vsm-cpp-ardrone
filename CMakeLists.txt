cmake_minimum_required (VERSION 2.6)

project(vsm-cpp-ardrone)

# Deal with path to SDK
if (NOT DEFINED VSM_SDK_DIR)
    if (DEFINED ENV{VSM_SDK_DIR})
        set(VSM_SDK_DIR $ENV{VSM_SDK_DIR})
    else()
        # Default SDK path
        if(CMAKE_SYSTEM_NAME MATCHES "Windows")
            set(VSM_SDK_DIR "C:/Program Files/vsm_sdk")
        else()
            set(VSM_SDK_DIR "/opt/vsm_sdk")
        endif()
    endif()
endif()

include("${VSM_SDK_DIR}/lib/make/vsm.cmake")

Build_vsm_config(${CMAKE_SOURCE_DIR}/vsm.conf)

if (NOT DEFINED COMMON_SOURCES)
    set(COMMON_SOURCES "../vsm-cpp-common")
endif()

include_directories (include "${COMMON_SOURCES}/include")

file(GLOB SOURCES "src/*.cpp" "${COMMON_SOURCES}/src/*mavlink*.cpp") 

# Add the executable
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME} ${VSM_LIBS})

Add_install_target()
