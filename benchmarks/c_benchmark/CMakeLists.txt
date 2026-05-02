cmake_minimum_required(VERSION 3.10)
project(c_benchmark C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# Path to the bootstrapped WebARKitLib source (relative to THIS file)
set(AR_LIB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/WebARKitLib/lib/SRC")
set(AR_LIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/WebARKitLib/include")
set(JPEG_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/jpeg-9f")

# Add include directories
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${AR_LIB_INCLUDE_DIR}"
    "${JPEG_SRC_DIR}"
)

# libjpeg-9f core sources (bootstrapped via libraries.json)
set(JPEG_SOURCES
    "${JPEG_SRC_DIR}/jaricom.c"
    "${JPEG_SRC_DIR}/jcapimin.c"
    "${JPEG_SRC_DIR}/jcapistd.c"
    "${JPEG_SRC_DIR}/jcarith.c"
    "${JPEG_SRC_DIR}/jccoefct.c"
    "${JPEG_SRC_DIR}/jccolor.c"
    "${JPEG_SRC_DIR}/jcdctmgr.c"
    "${JPEG_SRC_DIR}/jchuff.c"
    "${JPEG_SRC_DIR}/jcinit.c"
    "${JPEG_SRC_DIR}/jcmainct.c"
    "${JPEG_SRC_DIR}/jcmarker.c"
    "${JPEG_SRC_DIR}/jcmaster.c"
    "${JPEG_SRC_DIR}/jcomapi.c"
    "${JPEG_SRC_DIR}/jcparam.c"
    "${JPEG_SRC_DIR}/jcprepct.c"
    "${JPEG_SRC_DIR}/jcsample.c"
    "${JPEG_SRC_DIR}/jctrans.c"
    "${JPEG_SRC_DIR}/jdapimin.c"
    "${JPEG_SRC_DIR}/jdapistd.c"
    "${JPEG_SRC_DIR}/jdarith.c"
    "${JPEG_SRC_DIR}/jdatadst.c"
    "${JPEG_SRC_DIR}/jdatasrc.c"
    "${JPEG_SRC_DIR}/jdcoefct.c"
    "${JPEG_SRC_DIR}/jdcolor.c"
    "${JPEG_SRC_DIR}/jddctmgr.c"
    "${JPEG_SRC_DIR}/jdhuff.c"
    "${JPEG_SRC_DIR}/jdinput.c"
    "${JPEG_SRC_DIR}/jdmainct.c"
    "${JPEG_SRC_DIR}/jdmarker.c"
    "${JPEG_SRC_DIR}/jdmaster.c"
    "${JPEG_SRC_DIR}/jdmerge.c"
    "${JPEG_SRC_DIR}/jdpostct.c"
    "${JPEG_SRC_DIR}/jdsample.c"
    "${JPEG_SRC_DIR}/jdtrans.c"
    "${JPEG_SRC_DIR}/jerror.c"
    "${JPEG_SRC_DIR}/jfdctflt.c"
    "${JPEG_SRC_DIR}/jfdctfst.c"
    "${JPEG_SRC_DIR}/jfdctint.c"
    "${JPEG_SRC_DIR}/jidctflt.c"
    "${JPEG_SRC_DIR}/jidctfst.c"
    "${JPEG_SRC_DIR}/jidctint.c"
    "${JPEG_SRC_DIR}/jmemmgr.c"
    "${JPEG_SRC_DIR}/jmemnobs.c"
    "${JPEG_SRC_DIR}/jquant1.c"
    "${JPEG_SRC_DIR}/jquant2.c"
    "${JPEG_SRC_DIR}/jutils.c"
)

# Core AR files
set(AR_CORE_SOURCES
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
)

# KPM / FreakMatcher C++ sources
set(KPM_SOURCES
    "${AR_LIB_SRC_DIR}/KPM/kpmHandle.cpp"
    "${AR_LIB_SRC_DIR}/KPM/kpmRefDataSet.cpp"
    "${AR_LIB_SRC_DIR}/KPM/kpmMatching.cpp"
    "${AR_LIB_SRC_DIR}/KPM/kpmResult.cpp"
    "${AR_LIB_SRC_DIR}/KPM/kpmUtil.cpp"
    "${AR_LIB_SRC_DIR}/KPM/kpmFopen.c"
    # FreakMatcher
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/detectors/DoG_scale_invariant_detector.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/detectors/gaussian_scale_space_pyramid.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/detectors/gradients.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/detectors/orientation_assignment.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/detectors/pyramid.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/facade/visual_database_facade.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/framework/date_time.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/framework/image.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/framework/logger.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/framework/timers.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/matchers/freak.cpp"
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher/matchers/hough_similarity_voting.cpp"
)

# Common compile definitions
set(COMMON_DEFS AR_LITTLE_ENDIAN BINARY_FEATURE=1)

if(WIN32)
    list(APPEND COMMON_DEFS _WIN32)
endif()

# ── Existing marker detection benchmark ──
add_executable(c_benchmark ${AR_CORE_SOURCES} "main.c")
target_compile_definitions(c_benchmark PRIVATE ${COMMON_DEFS})
if(WIN32 AND MSVC)
    target_compile_options(c_benchmark PRIVATE /W3 /Z7)
elseif(NOT WIN32)
    target_link_libraries(c_benchmark m)
endif()

# ── Pattern-extraction diagnostic dumper (issue #103) ──
# Sibling of c_benchmark; runs detection once, re-extracts the matched patch
# via arPattGetImage, and dumps it for byte-comparison with the Rust port.
# See ../data/README.md and ../../docs/issue-103-fix-plan.md.
add_executable(dump_patt ${AR_CORE_SOURCES} "dump_patt.c")
target_compile_definitions(dump_patt PRIVATE ${COMMON_DEFS})
if(WIN32 AND MSVC)
    target_compile_options(dump_patt PRIVATE /W3 /Z7)
elseif(NOT WIN32)
    target_link_libraries(dump_patt m)
endif()

# ── KPM targets (fixture generator + benchmark) ──
# These require libjpeg-9f (bootstrapped) and are NOT built by default.
# Build manually with: cmake --build . --target kpm_dump_fixtures
# Stubs provide file_utils symbols (cat, test_d, mkdir_p) that main.c
# defines itself, so they are only added to the KPM targets.
set(STUBS "${CMAKE_CURRENT_SOURCE_DIR}/stubs.c")

add_executable(kpm_dump_fixtures EXCLUDE_FROM_ALL ${AR_CORE_SOURCES} ${KPM_SOURCES} ${JPEG_SOURCES} ${STUBS} "kpm_dump_fixtures.cpp")
target_compile_definitions(kpm_dump_fixtures PRIVATE ${COMMON_DEFS} HAVE_LIBJPEG)
target_include_directories(kpm_dump_fixtures PRIVATE
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher"
)
if(WIN32 AND MSVC)
    target_compile_options(kpm_dump_fixtures PRIVATE /W3 /Z7)
elseif(NOT WIN32)
    target_link_libraries(kpm_dump_fixtures m)
endif()

# ── KPM performance benchmark ──
add_executable(kpm_bench EXCLUDE_FROM_ALL ${AR_CORE_SOURCES} ${KPM_SOURCES} ${JPEG_SOURCES} ${STUBS} "kpm_bench.cpp")
target_compile_definitions(kpm_bench PRIVATE ${COMMON_DEFS} HAVE_LIBJPEG)
target_include_directories(kpm_bench PRIVATE
    "${AR_LIB_SRC_DIR}/KPM/FreakMatcher"
)
if(WIN32 AND MSVC)
    target_compile_options(kpm_bench PRIVATE /W3 /Z7)
elseif(NOT WIN32)
    target_link_libraries(kpm_bench m)
endif()
