include(${CMAKE_CURRENT_LIST_DIR}/../../HMIShared/CMakeLists.txt)
set(HMI_TERMINAL_SRC
    ${CMAKE_CURRENT_LIST_DIR}/HMITerminalPlugin.cpp
    ${HMI_STRUCTURES_SRC})
ADD_TERMINAL_PLUGIN(hmiTerminalPlugin "${HMI_TERMINAL_SRC}")