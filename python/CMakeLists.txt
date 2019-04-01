cmake_minimum_required(VERSION 3.10.0)

set(INSTALL_PATH ${CMAKE_INSTALL_DATADIR}/cwipc_codec/python)

install(FILES test_cwipc_codec.py DESTINATION ${INSTALL_PATH})

find_package(Python3 COMPONENTS Interpreter)

# Unfortunately Ubuntu 18.04 doesn't have the Python3 package (introduced in cmake 3.12 or so). Work around it.
if(NOT Python3_FOUND)
    find_program(PY3INTERPRETER python3)
    if(PY3INTERPRETER)
        set(Python3_EXECUTABLE ${PY3INTERPRETER})
        set(Python3_FOUND TRUE)
    endif()
endif()
if(Python3_FOUND)
	add_test(NAME run_python_tests
		COMMAND ${Python3_EXECUTABLE} test_cwipc_codec.py
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	
	# Ensure the cwipc_codec dynamic library and cwipc python module can be found by the tests
	if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set_property(TEST run_python_tests PROPERTY ENVIRONMENT DYLD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{DYLD_LIBRARY_PATH})
		set_property(TEST run_python_tests APPEND PROPERTY ENVIRONMENT PYTHONPATH=${cwipc_util_PYTHON_DIRS}:$ENV{PYTHONPATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set_property(TEST run_python_tests PROPERTY ENVIRONMENT LD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:${cwipc_util_LIBRARY_DIRS}:$ENV{LD_LIBRARY_PATH})
		set_property(TEST run_python_tests APPEND PROPERTY ENVIRONMENT PYTHONPATH=${cwipc_util_PYTHON_DIRS}:$ENV{PYTHONPATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		# Jack doesn't understand why modifying PATH doesn't seem to work on Windows...
		# Jack doesn't understand why we need the "Release" here, which should already be in CMAKE_RUNTIME_OUTPUT_DIRECTORY...
		set_property(TEST run_python_tests PROPERTY ENVIRONMENT CWIPC_TEST_DLL=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release/cwipc_codec.dll)
		set_property(TEST run_python_tests APPEND PROPERTY ENVIRONMENT PYTHONPATH=${cwipc_util_PYTHON_DIRS};$ENV{PYTHONPATH})
	endif()
endif()
