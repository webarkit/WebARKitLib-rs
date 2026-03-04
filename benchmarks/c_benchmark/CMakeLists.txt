cmake_minimum_required(VERSION 3.10)
project(c_benchmark C)

set(CMAKE_C_STANDARD 99)

# Path to the bootstrapped WebARKitLib source (relative to THIS file)
set(AR_LIB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/WebARKitLib/lib/SRC")
set(AR_LIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/WebARKitLib/include")

# Add include directories
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${AR_LIB_INCLUDE_DIR}"
)

# Core AR files
set(SOURCES
    "${AR_LIB_SRC_DIR}/AR/ar3DCreateHandle.c"
    "${AR_LIB_SRC_DIR}/AR/ar3DUtil.c"
    "${AR_LIB_SRC_DIR}/AR/arCreateHandle.c"
    "${AR_LIB_SRC_DIR}/AR/arDetectMarker.c"
    "${AR_LIB_SRC_DIR}/AR/arDetectMarker2.c"
    "${AR_LIB_SRC_DIR}/AR/arFilterTransMat.c"
    "${AR_LIB_SRC_DIR}/AR/arGetLine.c"
    "${AR_LIB_SRC_DIR}/AR/arGetMarkerInfo.c"
    "${AR_LIB_SRC_DIR}/AR/arGetTransMat.c"
    "${AR_LIB_SRC_DIR}/AR/arImageProc.c"
    "${AR_LIB_SRC_DIR}/AR/arLabeling.c"
    
    # Labeling Sub
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubDBIC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubDBRC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubDBZ.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubDWIC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubDWRC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubDWZ.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubEBIC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubEBRC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubEBZ.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubEWIC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubEWRC.c"
    "${AR_LIB_SRC_DIR}/AR/arLabelingSub/arLabelingSubEWZ.c"

    "${AR_LIB_SRC_DIR}/AR/arPattAttach.c"
    "${AR_LIB_SRC_DIR}/AR/arPattCreateHandle.c"
    "${AR_LIB_SRC_DIR}/AR/arPattGetID.c"
    "${AR_LIB_SRC_DIR}/AR/arPattLoad.c"
    "${AR_LIB_SRC_DIR}/AR/arUtil.c"
    
    # Math/Matrix/Param
    "${AR_LIB_SRC_DIR}/AR/mAlloc.c"
    "${AR_LIB_SRC_DIR}/AR/mFree.c"
    "${AR_LIB_SRC_DIR}/AR/mAllocDup.c"
    "${AR_LIB_SRC_DIR}/AR/mDup.c"
    "${AR_LIB_SRC_DIR}/AR/mAllocTrans.c"
    "${AR_LIB_SRC_DIR}/AR/mTrans.c"
    "${AR_LIB_SRC_DIR}/AR/mAllocMul.c"
    "${AR_LIB_SRC_DIR}/AR/mMul.c"
    "${AR_LIB_SRC_DIR}/AR/mAllocInv.c"
    "${AR_LIB_SRC_DIR}/AR/mInv.c"
    "${AR_LIB_SRC_DIR}/AR/mSelfInv.c"
    "${AR_LIB_SRC_DIR}/AR/mAllocUnit.c"
    "${AR_LIB_SRC_DIR}/AR/mUnit.c"
    "${AR_LIB_SRC_DIR}/AR/mDisp.c"
    "${AR_LIB_SRC_DIR}/AR/mDet.c"
    "${AR_LIB_SRC_DIR}/AR/mPCA.c"
    "${AR_LIB_SRC_DIR}/AR/vAlloc.c"
    "${AR_LIB_SRC_DIR}/AR/vDisp.c"
    "${AR_LIB_SRC_DIR}/AR/vFree.c"
    "${AR_LIB_SRC_DIR}/AR/vHouse.c"
    "${AR_LIB_SRC_DIR}/AR/vInnerP.c"
    "${AR_LIB_SRC_DIR}/AR/vTridiag.c"
    "${AR_LIB_SRC_DIR}/AR/paramGetPerspective.c"
    "${AR_LIB_SRC_DIR}/AR/paramClear.c"
    "${AR_LIB_SRC_DIR}/AR/paramDecomp.c"
    "${AR_LIB_SRC_DIR}/AR/paramDistortion.c"
    "${AR_LIB_SRC_DIR}/AR/paramChangeSize.c"
    "${AR_LIB_SRC_DIR}/AR/paramFile.c"
    "${AR_LIB_SRC_DIR}/AR/paramLT.c"
    "${AR_LIB_SRC_DIR}/AR/paramGL.c"
    "${AR_LIB_SRC_DIR}/AR/paramDisp.c"

    # ARICP
    "${AR_LIB_SRC_DIR}/ARICP/icpCore.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpHandle.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpPoint.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpPointRobust.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpStereoHandle.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpStereoPoint.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpStereoPointRobust.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpCalibStereo.c"
    "${AR_LIB_SRC_DIR}/ARICP/icpUtil.c"
    
    # ARUtil
    "${AR_LIB_SRC_DIR}/ARUtil/log.c"
    # "${AR_LIB_SRC_DIR}/ARUtil/file_utils.c" # Requires zlib
    # "${AR_LIB_SRC_DIR}/ARUtil/time.c"

    # Main benchmark file
    "main.c"
)

# Output executable
add_executable(c_benchmark ${SOURCES})

# Target compiles with local config.h
target_compile_definitions(c_benchmark PRIVATE _WIN32 AR_LITTLE_ENDIAN)

if(MSVC)
    target_compile_options(c_benchmark PRIVATE /W3 /Z7)
endif()
