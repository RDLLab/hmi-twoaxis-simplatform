include(${CMAKE_CURRENT_LIST_DIR}/../../HMIShared/CMakeLists.txt)
set(HMI_REWARD_SRC
    ${CMAKE_CURRENT_LIST_DIR}/HMIRewardPlugin.cpp
    ${HMI_STRUCTURES_SRC})
ADD_REWARD_PLUGIN(hmiRewardPlugin "${HMI_REWARD_SRC}")