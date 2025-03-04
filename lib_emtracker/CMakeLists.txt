# Define the project and its version
project(EMTlib VERSION 1.1.2)

# Set the release date manually
set(PROJECT_RELEASE_DATE "2025-01-15")

# List the EMT library source files into a variable
set(EMT_LIB_SOURCES 
    src/EMTracker.cpp
    src/RigidTransformation.cpp
    )

# Create a static library named EMTlib using the listed source files
add_library(${PROJECT_NAME} STATIC ${EMT_LIB_SOURCES})

# Add the necessary include directories for the EMTlib library
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ndi_api/include)

# turn off build warnings from the NDI library
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unknown-pragmas")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-variable")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-but-set-variable")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-parameter")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-sign-compare")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-overflow")

set(BLA_STATIC ON)  # use static linkage

# for dynamic access to the conf directory in EMTracker.cpp
target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/config/")
target_compile_definitions(${PROJECT_NAME} PRIVATE outPutFiles_Directory="${CMAKE_SOURCE_DIR}/OutputFiles/")
target_compile_definitions(${PROJECT_NAME} PRIVATE Log_directory="${CMAKE_SOURCE_DIR}/OutputFiles/Log/")

# linking NDI's shared library
add_subdirectory(ndi_api) 
target_link_libraries(${PROJECT_NAME} PUBLIC NDI)

# Find and link Blaze package
set(blaze_DIR /usr/local/include/blaze)
find_package(blaze REQUIRED PATH ${blaze_DIR})
if( blaze_FOUND )
   message(STATUS ">>> Blaze library has been found!")
   target_link_libraries(${PROJECT_NAME} PRIVATE blaze::blaze)
elseif(NOT Blaze_FOUND)
    message(FATAL_ERROR ">>> Blaze has not been found!")
endif()

# Find and link NLopt package
find_package(NLopt)
if (NLopt_FOUND)
    message(STATUS ">>> NLopt library has been found!")
    # include_directories(${NLopt_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE nlopt m)
    # target_link_libraries(EMTlib PUBLIC NLopt::nlopt)
else (NOT NLopt_FOUND)
    message(FATAL_ERROR ">>> NLopt library has not been found")
endif()

# Find and link LAPACK package
# LPACK is not directly used in the code but it is required by blaze 
find_package(LAPACK REQUIRED)
if (LAPACK_FOUND)
    message(STATUS ">>> LAPACK has been found!")
    include_directories(${LAPACK_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LAPACK_LIBRARIES})
else (NOT LAPACK_FOUND)
    message(FATAL_ERROR ">>> LAPACK API has not been found")
endif()

# Find and link BLAS package
set(BLA_VENDOR OpenBLAS)
find_package(BLAS)
if (BLAS_FOUND)
    message(STATUS "BLAS has been found!")
    # include_directories(${BLAS_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${BLAS_LIBRARIES})
elseif(NOT BLAS_FOUND)
    message(FATAL_ERROR "BLAS library has not been found")
endif()

# linking gfortran library
target_link_libraries(${PROJECT_NAME} PRIVATE gfortran)

find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)