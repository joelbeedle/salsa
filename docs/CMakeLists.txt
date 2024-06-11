project(swarm-sim)
# Find the Doxygen package
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_INPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

    set(DOXYGEN_CONFIGURED_INPUT ${DOXYGEN_OUTPUT_DIR}/doxyfile)
    configure_file(${DOXYGEN_INPUT_DIR}/doxyfile.in ${DOXYGEN_CONFIGURED_INPUT} @ONLY)
    configure_file(${DOXYGEN_INPUT_DIR}/custom.css ${DOXYGEN_OUTPUT_DIR}/doxygen.css COPYONLY)

    message("Doxygen build started.")

    add_custom_target(docs 
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIGURED_INPUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else()
    message("Doxygen needs to be installed to generate the documentation")
endif()
