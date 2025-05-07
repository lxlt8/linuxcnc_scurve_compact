cmake_minimum_required(VERSION 3.16)

# Define a variable for the project name
set(PROJECT_NAME ethercat_tool)
project(${PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set SRC_DIR to the current directory
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

include_directories(${SRC_DIR}/../master)
include_directories(${SRC_DIR}/../include)
include_directories(${SRC_DIR}/) # Config.h

add_executable(${PROJECT_NAME} main.cpp
    CommandAlias.cpp
    CommandAlias.h
    CommandConfig.cpp
    CommandConfig.h
    Command.cpp
    CommandCrc.cpp
    CommandCrc.h
    CommandCStruct.cpp
    CommandCStruct.h
    CommandData.cpp
    CommandData.h
    CommandDebug.cpp
    CommandDebug.h
    CommandDomains.cpp
    CommandDomains.h
    CommandDownload.cpp
    CommandDownload.h
    CommandEoe.cpp
    CommandEoe.h
    CommandFoeRead.cpp
    CommandFoeRead.h
    CommandFoeWrite.cpp
    CommandFoeWrite.h
    CommandGraph.cpp
    CommandGraph.h
    Command.h
    CommandIp.cpp
    CommandIp.h
    CommandMaster.cpp
    CommandMaster.h
    CommandPdos.cpp
    CommandPdos.h
    CommandRegRead.cpp
    CommandRegRead.h
    CommandRegWrite.cpp
    CommandRegWrite.h
    CommandRescan.cpp
    CommandRescan.h
    CommandSlaves.cpp
    CommandSlaves.h
    CommandSiiRead.cpp
    CommandSiiRead.h
    CommandSiiWrite.cpp
    CommandSiiWrite.h
    CommandStates.cpp
    CommandStates.h
    CommandSdos.cpp
    CommandSdos.h
    CommandUpload.cpp
    CommandUpload.h
    CommandVersion.cpp
    CommandVersion.h
    CommandXml.cpp
    CommandXml.h
    DataTypeHandler.cpp
    DataTypeHandler.h
    MasterDevice.cpp
    MasterDevice.h
    NumberListParser.cpp
    NumberListParser.h
    SdoCommand.cpp
    SdoCommand.h
    sii_crc.cpp
    sii_crc.h
    FoeCommand.h
    FoeCommand.cpp
    CommandSoeRead.h
    CommandSoeRead.cpp
    CommandSoeWrite.h
    CommandSoeWrite.cpp
    SoeCommand.h
    SoeCommand.cpp
    # ${SRC_DIR}/../globals.h
    # ${SRC_DIR}/../config.h
    main.cpp
)

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
