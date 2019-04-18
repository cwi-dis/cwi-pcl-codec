cmake_minimum_required(VERSION 3.10.0)

set(INSTALL_PATH ${CMAKE_INSTALL_DATADIR}/cwipc_codec/tests)

install(DIRECTORY fixtures DESTINATION ${INSTALL_PATH})
