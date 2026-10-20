# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.22)

# Crucible Engine: a polyphonic synthesis engine as a standalone C++20 static library with no framework dependencies.
# Build standalone (library, tests, examples) or consume it from another project with add_subdirectory().
# The project version is the single source of the library version: it generates crucible/Version.h, which backs
# crucible::versionString(), crucible_version() and --version of the tools. CRUCIBLE_API_VERSION (crucible.h) is the
# separate C ABI counter.
project(CrucibleEngine VERSION 0.2.0 LANGUAGES C CXX)

include(GNUInstallDirs)

set(CRUCIBLE_ENGINE_TOP_LEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CRUCIBLE_ENGINE_TOP_LEVEL ON)
endif()

option(CRUCIBLE_ENGINE_BUILD_TESTS "Build the CrucibleEngine tests" ${CRUCIBLE_ENGINE_TOP_LEVEL})
option(CRUCIBLE_ENGINE_BUILD_EXAMPLES "Build the CrucibleEngine examples" ${CRUCIBLE_ENGINE_TOP_LEVEL})
option(CRUCIBLE_ENGINE_INSTALL "Generate install rules and the CrucibleEngine package files" ${CRUCIBLE_ENGINE_TOP_LEVEL})
# Shared-library loading of third-party modules (crucible_load_module_library). Even when compiled in, a host has to
# allow it at run time; switch it off here for builds that must not carry dlopen / LoadLibrary at all.
option(CRUCIBLE_ENGINE_MODULE_LOADING "Compile support for loading module shared libraries" ON)

# Sanitizer builds for CI: "address;undefined" or "thread" (see tools/ci.sh).
set(CRUCIBLE_ENGINE_SANITIZE "" CACHE STRING "Sanitizers to enable: address, undefined, thread (semicolon separated)")
if(CRUCIBLE_ENGINE_SANITIZE AND MSVC)
    message(FATAL_ERROR "CRUCIBLE_ENGINE_SANITIZE needs gcc or clang (MSVC only offers AddressSanitizer via /fsanitize=address)")
elseif(CRUCIBLE_ENGINE_SANITIZE)
    string(REPLACE ";" "," CRUCIBLE_SANITIZE_FLAGS "${CRUCIBLE_ENGINE_SANITIZE}")
    add_compile_options(-fsanitize=${CRUCIBLE_SANITIZE_FLAGS} -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=${CRUCIBLE_SANITIZE_FLAGS})
endif()

# Vector kernels: CRUCIBLE_FORCE_SCALAR runs the plain scalar code everywhere (CI uses it to exercise the fallback that
# platforms without NEON / SSE take); CRUCIBLE_X86_AVX2 compiles x86 builds for AVX2 (gathers) and FMA.
option(CRUCIBLE_FORCE_SCALAR "Disable the SIMD kernels and use the scalar code paths" OFF)
option(CRUCIBLE_X86_AVX2 "Compile for AVX2 on x86 (vector gathers)" OFF)

if(MSVC)
    # UTF-8 sources, large translation units in the tests, and the standard-conforming preprocessor and two-phase lookup.
    add_compile_options(/utf-8 /bigobj /permissive- /W3 /wd4244 /wd4267 /wd4305)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES NOMINMAX)
endif()

if(CRUCIBLE_ENGINE_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
endif()

# crucible/Version.h is generated into the build tree.
set(CRUCIBLE_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
configure_file(cmake/Version.h.in "${CRUCIBLE_GENERATED_DIR}/crucible/Version.h" @ONLY)

# Modules: every file under src/modules is compiled; a module registers itself with one line in
# src/modules/Modules.cpp.
file(GLOB_RECURSE CRUCIBLE_ENGINE_MODULE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/modules/*.cpp")

add_library(crucible_engine STATIC
    src/Arpeggiator.cpp
    src/ArpPatternIO.cpp
    src/AudioFile.cpp
    src/CApi.cpp
    src/CcMap.cpp
    src/Clips.cpp
    src/Cloud.cpp
    src/MinBlep.cpp
    src/Oscillator.cpp
    src/Oversampler.cpp
    src/Sample.cpp
    src/Saturation.cpp
    src/SaturationAvx.cpp
    src/Engine.cpp
    src/EngineViews.cpp
    src/EngineCore.cpp
    src/Fft.cpp
    src/Filters.cpp
    src/FilterBank.cpp
    src/filters/DiodeLadder.cpp
    src/filters/LadderBase.cpp
    src/filters/Ota4Pole.cpp
    src/filters/OtaCascade.cpp
    src/filters/PassiveHp.cpp
    src/filters/SallenKey35.cpp
    src/filters/SvfOta.cpp
    src/filters/TransistorLadder.cpp
    src/Fx.cpp
    src/Introspection.cpp
    src/FxExtra.cpp
    src/fx/FxCore.cpp
    src/fx/HalfBandAvx.cpp
    src/fx/Dynamics.cpp
    src/fx/Loudness.cpp
    src/fx/Maximizer.cpp
    src/fx/Clipper.cpp
    src/fx/ShaperCurve.cpp
    src/fx/AnalogMatchedFilters.cpp
    src/fx/LinearPhase.cpp
    src/fx/Crossover.cpp
    src/fx/Fdn.cpp
    src/fx/FractionalDelay.cpp
    src/fx/Bbd.cpp
    src/fx/EarlyReflections.cpp
    src/fx/PlateTank.cpp
    src/fx/AudioFftAdapter.cpp
    src/fx/PartitionedConvolver.cpp
    src/fx/CabinetModel.cpp
    src/fx/IrImport.cpp
    src/fx/IrEdit.cpp
    src/ParameterMap.cpp
    src/ParamText.cpp
    src/Patch.cpp
    src/PresetLibrary.cpp
    src/Scenes.cpp
    src/Sequencer.cpp
    src/SynthEngine.cpp
    src/Spectral.cpp
    src/Voice.cpp
    src/Tuning.cpp
    src/Version.cpp
    src/Views.cpp
    src/UserWavetables.cpp
    src/Wavetables.cpp
    src/Zones.cpp
    src/Json.cpp
    src/KeyBus.cpp
    src/KeyDetect.cpp
    src/KeyLock.cpp
    src/MasterAnalyzer.cpp
    src/MidiFile.cpp
    src/ModuleRack.cpp
    src/graph/Compiler.cpp
    src/graph/Nodes.cpp
    src/graph/Product.cpp
    src/graph/Program.cpp
    src/MotionRecorder.cpp
    src/Modulation.cpp
    src/Modulators.cpp
    src/sdk/Check.cpp
    src/sdk/CModuleApi.cpp
    src/sdk/ModuleSet.cpp
    src/sdk/SharedLibrary.cpp
    ${CRUCIBLE_ENGINE_MODULE_SOURCES}
    third_party/mutable/rings/resources.cc
    third_party/pffft/src/pffft.c
    third_party/pffft/src/pffft_common.c
    third_party/fftconvolver/Utilities.cpp)
add_library(crucible::engine ALIAS crucible_engine)
target_include_directories(crucible_engine
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CRUCIBLE_GENERATED_DIR}>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
# Vendored MIT code (third_party/mutable: tables and headers of the Rings / Elements ports; third_party/signalsmith-*:
# time stretch). Private to the library: no public header includes them.
target_include_directories(crucible_engine SYSTEM PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mutable"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/signalsmith-stretch/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/signalsmith-linear/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/fftconvolver")
set_source_files_properties(third_party/mutable/rings/resources.cc third_party/pffft/src/pffft.c third_party/pffft/src/pffft_common.c
    third_party/fftconvolver/Utilities.cpp
    PROPERTIES COMPILE_OPTIONS "$<IF:$<CXX_COMPILER_ID:MSVC>,/w,-w>")
# MSVC x64: the eight-lane modal bank, the four-lane second-order ADAA pair kernel and the eight-lane half-band dot products
# are the only files built for AVX; the library calls them only on CPUs with AVX, and their output is bit-identical to the
# SSE kernels (src/modules/oscillator/ModalBankAvx.h, src/SaturationAvx.h, include/crucible/fx/HalfBandChain.h).
if(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND NOT CMAKE_SYSTEM_PROCESSOR MATCHES "ARM|arm")
    set_source_files_properties(src/modules/oscillator/ModalBankAvx.cpp src/SaturationAvx.cpp src/fx/HalfBandAvx.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX")
endif()
# FFT backend behind crucible/Fft.h: AUTO is Accelerate on Apple and pffft (vendored, SIMD) elsewhere; PORTABLE is the
# double-precision radix-2 code of our own. pffft is always compiled so that the tests can compare it with the reference.
set(CRUCIBLE_ENGINE_FFT_BACKEND "AUTO" CACHE STRING "FFT backend: AUTO, ACCELERATE, PFFFT or PORTABLE")
set_property(CACHE CRUCIBLE_ENGINE_FFT_BACKEND PROPERTY STRINGS AUTO ACCELERATE PFFFT PORTABLE)
target_include_directories(crucible_engine SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pffft/include/pffft")
target_compile_definitions(crucible_engine PRIVATE PFFFT_STATIC_DEFINE=1)
# The engine is a static library: its C API carries no import / export decoration (CRUCIBLE_API in crucible.h). With
# CRUCIBLE_ENGINE_SHARED the same sources are compiled for the shared library instead (see the end of the target section).
option(CRUCIBLE_ENGINE_SHARED "Build the C API as a shared library (libcrucible) for FFI hosts such as the Python package" OFF)
if(CRUCIBLE_ENGINE_SHARED)
    target_compile_definitions(crucible_engine PUBLIC CRUCIBLE_BUILD_SHARED=1)
    set_target_properties(crucible_engine PROPERTIES C_VISIBILITY_PRESET hidden CXX_VISIBILITY_PRESET hidden
                          VISIBILITY_INLINES_HIDDEN ON)
else()
    target_compile_definitions(crucible_engine PUBLIC CRUCIBLE_STATIC=1)
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64|ARM64|armv8")
    set_source_files_properties(third_party/pffft/src/pffft.c PROPERTIES COMPILE_DEFINITIONS PFFFT_ENABLE_NEON=1)
endif()
if(CRUCIBLE_ENGINE_FFT_BACKEND STREQUAL "PFFFT")
    target_compile_definitions(crucible_engine PRIVATE CRUCIBLE_FFT_PFFFT=1)
elseif(CRUCIBLE_ENGINE_FFT_BACKEND STREQUAL "PORTABLE")
    target_compile_definitions(crucible_engine PRIVATE CRUCIBLE_FFT_PORTABLE=1)
elseif(CRUCIBLE_ENGINE_FFT_BACKEND STREQUAL "ACCELERATE" AND NOT APPLE)
    message(FATAL_ERROR "CRUCIBLE_ENGINE_FFT_BACKEND=ACCELERATE needs an Apple platform")
endif()
set_target_properties(crucible_engine PROPERTIES VERSION ${PROJECT_VERSION} EXPORT_NAME engine)
target_compile_features(crucible_engine PUBLIC cxx_std_20)
set_target_properties(crucible_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(APPLE)
    target_link_libraries(crucible_engine PRIVATE "-framework Accelerate")
endif()
if(CRUCIBLE_FORCE_SCALAR)
    target_compile_definitions(crucible_engine PUBLIC CRUCIBLE_FORCE_SCALAR=1)
endif()
if(CRUCIBLE_X86_AVX2 AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(crucible_engine PUBLIC -mavx2 -mfma)
endif()
find_package(Threads REQUIRED)
target_link_libraries(crucible_engine PUBLIC Threads::Threads)
if(CRUCIBLE_ENGINE_MODULE_LOADING)
    target_link_libraries(crucible_engine PUBLIC ${CMAKE_DL_LIBS})
else()
    target_compile_definitions(crucible_engine PRIVATE CRUCIBLE_NO_MODULE_LOADING=1)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(crucible_engine PRIVATE -Wall -Wextra -Wshadow -Wno-unused-parameter)
endif()

if(CRUCIBLE_ENGINE_BUILD_TESTS)
    enable_testing()
    add_executable(crucible_engine_tests
        tests/TestMain.cpp
        tests/EngineTests.cpp
        tests/FacadeTests.cpp
        tests/HostPlumbingTests.cpp
        tests/FuzzCrashTests.cpp
        tests/FuzzStateTests.cpp
        tests/FilterTests.cpp
        tests/FormantTableTests.cpp
        tests/KeyLockTests.cpp
        tests/FxTests.cpp
        tests/FxExtraTests.cpp
        tests/UserWaveTests.cpp
        tests/UiAccessorTests.cpp
        tests/ArpTests.cpp
        tests/ClipTests.cpp
        tests/ModulationTests.cpp
        tests/ModuleTests.cpp
        tests/OscillatorTests.cpp
        tests/PatchTests.cpp
        tests/PerformanceTests.cpp
        tests/PresetTests.cpp
        tests/PlatformTests.cpp
        tests/QaTests.cpp
        tests/SkeletonFullTests.cpp
        tests/BitExactTests.cpp
        tests/VoiceGraphTests.cpp
        tests/OscillatorModuleTests.cpp
        tests/MinBlepTests.cpp
        tests/PolyphonyTests.cpp
        tests/VoiceClickTests.cpp
        tests/OversamplerTests.cpp
        tests/QualityTests.cpp
        tests/AnalogFilterLadderTests.cpp
        tests/AnalogFilterSkTests.cpp
        tests/VoiceQuadTests.cpp
        tests/AnalogMeasureTests.cpp
        tests/FxMeasureTests.cpp
        tests/FxFixTests.cpp
        tests/TuningOracleTests.cpp
        tests/MeasureEngineTests.cpp
        tests/SimdTests.cpp
        tests/VocoderTests.cpp
        tests/WavetableMemoryTests.cpp
        tests/SampleSliceTests.cpp
        tests/CloudDrawbarTests.cpp
        tests/UnisonModulationTests.cpp
        tests/VectorTests.cpp
        tests/SaturationTests.cpp
        tests/SaturationShapesTests.cpp
        tests/ZoneTests.cpp
        tests/ProductTests.cpp
        tests/GraphTests.cpp
        tests/IntrospectionTests.cpp
        tests/AnalogOscillatorTests.cpp
        tests/ViewTests.cpp
        tests/FilterResponseTests.cpp
        tests/WavetableViewTests.cpp
        tests/DynamicsTests.cpp
        tests/EqProTests.cpp
        tests/CrossoverTests.cpp
        tests/ReverbAlgoTests.cpp
        tests/SdkTests.cpp
        tests/EnginesTests.cpp
        tests/AnalogVoiceTests.cpp
        tests/LoudnessTests.cpp
        tests/DelayProTests.cpp
        tests/ModulationFxTests.cpp
        tests/TransientDeEsserTests.cpp
        tests/ExciterTests.cpp
        tests/CabinetTests.cpp
        tests/MaximizerTests.cpp
        tests/ParamOrderTests.cpp
        tests/ModulatorTests.cpp
        tests/AudioRateModTests.cpp
        tests/PartitionedConvolverTests.cpp
        tests/IrImportTests.cpp
        tests/Utf8PathTests.cpp
        tests/ConvolutionTests.cpp
        tests/ColourDynamicsTests.cpp
        tests/ShaperTests.cpp
        tests/MultibandDriveTests.cpp
        tests/NoiseTiltTests.cpp
        tests/RcAttackCurveTests.cpp
        tests/SmootherSettleTests.cpp
        tests/CrusherTrackTests.cpp
        tests/SaturatorFilterTests.cpp)
    target_include_directories(crucible_engine_tests PRIVATE tests)
    # Vendored MIT tuning-library, used only as the oracle in tests/TuningOracleTests.cpp.
    target_include_directories(crucible_engine_tests SYSTEM PRIVATE third_party/tuning-library/include)
    target_compile_definitions(crucible_engine_tests PRIVATE CRUCIBLE_ENGINE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    # Vendored MIT libebur128 (C), used only as the loudness oracle in tests/LoudnessTests.cpp; never linked into the library.
    add_library(crucible_ebur128_oracle STATIC third_party/libebur128/ebur128.c)
    target_include_directories(crucible_ebur128_oracle PUBLIC third_party/libebur128)
    target_include_directories(crucible_ebur128_oracle SYSTEM PRIVATE third_party/libebur128/queue)
    set_target_properties(crucible_ebur128_oracle PROPERTIES C_STANDARD 99 POSITION_INDEPENDENT_CODE ON)
    target_compile_definitions(crucible_ebur128_oracle PRIVATE _USE_MATH_DEFINES) # MSVC: M_PI
    if(NOT WIN32)
        target_link_libraries(crucible_ebur128_oracle PUBLIC m)
    endif()
    if(NOT MSVC)
        target_compile_options(crucible_ebur128_oracle PRIVATE -w)
    endif()
    target_link_libraries(crucible_engine_tests PRIVATE crucible_ebur128_oracle)
    target_compile_definitions(crucible_engine_tests PRIVATE CRUCIBLE_ENGINE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/golden"
        CRUCIBLE_ENGINE_PRODUCTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/products")
    target_link_libraries(crucible_engine_tests PRIVATE crucible_engine)
    add_test(NAME crucible_engine_tests COMMAND crucible_engine_tests)

    # The C API, compiled as C.
    add_executable(crucible_capi_test tests/capi_test.c)
    target_link_libraries(crucible_capi_test PRIVATE crucible_engine)
    set_target_properties(crucible_capi_test PROPERTIES LINKER_LANGUAGE CXX)
    add_test(NAME crucible_capi_test COMMAND crucible_capi_test)

    # Tests that need a wider voice graph (the shape is fixed per process, so they get their own executable).
    add_executable(crucible_engine_graph_tests tests/GraphMain.cpp tests/VoiceGraphWideTests.cpp)
    target_include_directories(crucible_engine_graph_tests PRIVATE tests)
    target_link_libraries(crucible_engine_graph_tests PRIVATE crucible_engine)
    add_test(NAME crucible_engine_graph_tests COMMAND crucible_engine_graph_tests)

    # Feature audit: renders every module and parameter offline (see docs/FEATURE-AUDIT.md). Its own executable so
    # that the long sweep can be run on its own: crucible_engine_audit_tests [filter].
    add_executable(crucible_engine_audit_tests tests/TestMain.cpp tests/FeatureAuditTests.cpp)
    target_include_directories(crucible_engine_audit_tests PRIVATE tests)
    target_link_libraries(crucible_engine_audit_tests PRIVATE crucible_engine)
    add_test(NAME crucible_engine_audit_tests COMMAND crucible_engine_audit_tests)
    # The sweep is throughput bound (about 1100 s on a Ryzen 9 5900X / MSVC: 40 ms to build and prepare each of thousands of engines
    # at ~9500 page faults apiece); a ctest --timeout default of 900 s is too short for it.
    # ThreadSanitizer slows the sweep several times over, so its builds get a longer limit.
    if("thread" IN_LIST CRUCIBLE_ENGINE_SANITIZE)
        set(CRUCIBLE_AUDIT_TIMEOUT 10800)
    else()
        set(CRUCIBLE_AUDIT_TIMEOUT 3600)
    endif()
    set_tests_properties(crucible_engine_audit_tests PROPERTIES TIMEOUT ${CRUCIBLE_AUDIT_TIMEOUT})

    # Common acceptance of the older effects (comp_clean, comp_vca, gate2, eq_pro, reverb_algo, convolution, maximizer, clipper,
    # meter): rates, latency, zipper at host blocks of 1 / 7 / 512, reset and bypass. Own executable so that the long zipper
    # sweep does not slow the main suite. "OlderFxPass" skips the checks listed in kKnownFail (tests/OlderFxAcceptanceTests.cpp); "OlderFxKnownFail" runs only those with the strict
    # limits and is expected to fail (WILL_FAIL), so it turns red when an engine fix makes a listed check pass: delete the
    # entry then. "OlderFxAll" (run by hand) runs everything strictly.
    add_executable(crucible_engine_older_fx_tests tests/TestMain.cpp tests/OlderFxAcceptanceTests.cpp)
    target_include_directories(crucible_engine_older_fx_tests PRIVATE tests)
    target_compile_definitions(crucible_engine_older_fx_tests PRIVATE
        CRUCIBLE_OLDERFX_KNOWN_FAIL="${CMAKE_CURRENT_SOURCE_DIR}/tests/OlderFxKnownFail.txt")
    target_link_libraries(crucible_engine_older_fx_tests PRIVATE crucible_engine)
    add_test(NAME crucible_older_fx_acceptance COMMAND crucible_engine_older_fx_tests OlderFxPass)
    add_test(NAME crucible_older_fx_acceptance_known_fail COMMAND crucible_engine_older_fx_tests OlderFxKnownFail)
    set_tests_properties(crucible_older_fx_acceptance crucible_older_fx_acceptance_known_fail PROPERTIES TIMEOUT 3600)
    # An empty list (only comments) runs no check, so the known-fail run would pass and WILL_FAIL would turn it red: the
    # test then stays registered but is not inverted. The list is re-read when the file changes.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/OlderFxKnownFail.txt")
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/tests/OlderFxKnownFail.txt" CRUCIBLE_OLDERFX_KNOWN_ENTRIES REGEX "^[^#]")
    if(CRUCIBLE_OLDERFX_KNOWN_ENTRIES)
        set_tests_properties(crucible_older_fx_acceptance_known_fail PROPERTIES WILL_FAIL TRUE)
    endif()

    # CPU benchmark (not part of CTest): voices x unison x FX, prints the real-time factor.
    add_executable(crucible_engine_bench tests/Bench.cpp)
    target_include_directories(crucible_engine_bench PRIVATE tests)
    target_link_libraries(crucible_engine_bench PRIVATE crucible_engine)

    # FFT backend benchmark (P13): portable, pffft and the backend of Fft.h; not part of CTest (timing).
    add_executable(crucible_engine_fft_bench tests/FftBench.cpp)
    target_link_libraries(crucible_engine_fft_bench PRIVATE crucible_engine)

    # Baseline measurements of the filters and oscillators (docs/MEASUREMENTS.md); prints Markdown, not part of CTest.
    add_executable(crucible_engine_measure tests/MeasureBaseline.cpp)
    target_include_directories(crucible_engine_measure PRIVATE tests)
    target_link_libraries(crucible_engine_measure PRIVATE crucible_engine)

    # Baseline of the legacy effects (docs/MEASUREMENTS.md, "Effects"): prints Markdown, `check` is a CTest guard.
    add_executable(crucible_engine_fx_baseline tests/FxBaseline.cpp)
    target_include_directories(crucible_engine_fx_baseline PRIVATE tests)
    target_link_libraries(crucible_engine_fx_baseline PRIVATE crucible_engine)
    add_test(NAME crucible_engine_fx_baseline COMMAND crucible_engine_fx_baseline check)
endif()

option(CRUCIBLE_ENGINE_BUILD_TOOLS "Build crucible-render" ${CRUCIBLE_ENGINE_TOP_LEVEL})
if(CRUCIBLE_ENGINE_BUILD_TOOLS)
    add_executable(crucible-render tools/crucible-render/main.cpp)
    target_link_libraries(crucible-render PRIVATE crucible_engine)
    add_executable(crucible-measure tools/crucible-measure/main.cpp)
    target_link_libraries(crucible-measure PRIVATE crucible_engine)
    add_executable(crucible-module-check tools/crucible-module-check/main.cpp)
    target_link_libraries(crucible-module-check PRIVATE crucible_engine)
    if(CRUCIBLE_ENGINE_BUILD_TESTS)
        # Non-ASCII output folder: the tools must see their arguments as UTF-8 on Windows too.
        add_test(NAME crucible_render_unicode_path
                 COMMAND ${CMAKE_COMMAND} -DTOOL=$<TARGET_FILE:crucible-render>
                         -DWORK=${CMAKE_CURRENT_BINARY_DIR}/unicode_path_test
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/RenderUnicodePath.cmake)
        # Every built-in module through the module check (the CPU budget is left to a quiet machine).
        add_test(NAME crucible_module_check_builtin COMMAND crucible-module-check --builtin --quiet --no-cpu-limit)
    endif()
endif()

if(CRUCIBLE_ENGINE_BUILD_EXAMPLES)
    add_executable(render_wav examples/render_wav.cpp)
    target_link_libraries(render_wav PRIVATE crucible_engine)
    add_executable(render_c examples/c_api/render_c.c)
    target_link_libraries(render_c PRIVATE crucible_engine)
    if(NOT WIN32)
        target_link_libraries(render_c PRIVATE m)
    endif()
    set_target_properties(render_c PROPERTIES LINKER_LANGUAGE CXX)
    if(CRUCIBLE_ENGINE_BUILD_TESTS)
        enable_testing()
        add_test(NAME crucible_example_render_c COMMAND render_c "${CMAKE_CURRENT_BINARY_DIR}/render_c_test.wav")
    endif()

    # The instrument skeleton: the plug-in lifecycle on the C API with a fake host. It runs as a test.
    add_executable(instrument_skeleton examples/instrument_skeleton/main.c)
    target_link_libraries(instrument_skeleton PRIVATE crucible_engine)
    if(NOT WIN32)
        target_link_libraries(instrument_skeleton PRIVATE m)
    endif()
    set_target_properties(instrument_skeleton PROPERTIES LINKER_LANGUAGE CXX)
    if(CRUCIBLE_ENGINE_BUILD_TESTS)
        add_test(NAME instrument_skeleton
                 COMMAND instrument_skeleton "${CMAKE_CURRENT_SOURCE_DIR}/examples/instrument_skeleton/skeleton_product.json")
    endif()

    if(CRUCIBLE_ENGINE_BUILD_TESTS)
        # The skeleton with every module and feature (examples/instrument_skeleton/skeleton_full_product.json), through the C API only.
        add_test(NAME skeleton_all_features COMMAND crucible_engine_tests "Skeleton all features")
        set_tests_properties(skeleton_all_features PROPERTIES TIMEOUT 1800)
    endif()

    # Third-party module examples: shared libraries that use only the module headers, never the engine library.
    # crucible_example_c_module_future declares an ABI version this engine does not speak (a loading test).
    set(CRUCIBLE_EXAMPLE_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples/third_party_module")
    foreach(module IN ITEMS crucible_example_c_module crucible_example_c_module_future crucible_example_cpp_module)
        if(module STREQUAL crucible_example_cpp_module)
            add_library(${module} MODULE "${CRUCIBLE_EXAMPLE_MODULE_DIR}/cpp_module.cpp")
            target_compile_features(${module} PRIVATE cxx_std_20)
        else()
            add_library(${module} MODULE "${CRUCIBLE_EXAMPLE_MODULE_DIR}/c_module.c")
            if(NOT WIN32)
                target_link_libraries(${module} PRIVATE m)
            endif()
        endif()
        target_include_directories(${module} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
        set_target_properties(${module} PROPERTIES PREFIX "" C_VISIBILITY_PRESET hidden CXX_VISIBILITY_PRESET hidden
                                                   VISIBILITY_INLINES_HIDDEN ON)
    endforeach()
    target_compile_definitions(crucible_example_c_module_future PRIVATE EXAMPLE_ABI_VERSION=999)
    if(CRUCIBLE_ENGINE_BUILD_TESTS AND CRUCIBLE_ENGINE_MODULE_LOADING)
        add_dependencies(crucible_engine_tests crucible_example_c_module crucible_example_c_module_future
                         crucible_example_cpp_module)
        target_compile_definitions(crucible_engine_tests PRIVATE
            CRUCIBLE_TEST_C_MODULE="$<TARGET_FILE:crucible_example_c_module>"
            CRUCIBLE_TEST_FUTURE_MODULE="$<TARGET_FILE:crucible_example_c_module_future>"
            CRUCIBLE_TEST_CPP_MODULE="$<TARGET_FILE:crucible_example_cpp_module>")
        if(TARGET crucible-module-check)
            add_test(NAME crucible_module_check_examples
                COMMAND crucible-module-check --no-cpu-limit $<TARGET_FILE:crucible_example_c_module>
                        $<TARGET_FILE:crucible_example_cpp_module>)
            add_test(NAME crucible_module_check_future_abi
                COMMAND crucible-module-check $<TARGET_FILE:crucible_example_c_module_future>)
            set_tests_properties(crucible_module_check_future_abi PROPERTIES PASS_REGULAR_EXPRESSION "ABI version 999")
        endif()
    endif()
endif()

# Optional shared library of the C API (libcrucible.dylib / .so / crucible.dll). Configure a separate build directory with
# -DCRUCIBLE_ENGINE_SHARED=ON: the engine sources are then compiled with CRUCIBLE_BUILD_SHARED and hidden visibility, so
# only the functions marked CRUCIBLE_API in crucible.h are exported. The default (static) build is unchanged.
if(CRUCIBLE_ENGINE_SHARED)
    set(CRUCIBLE_SHARED_STUB "${CMAKE_CURRENT_BINARY_DIR}/crucible_shared_stub.cpp")
    file(WRITE "${CRUCIBLE_SHARED_STUB}" "// Generated: the shared library is built from the engine archive.\n")
    add_library(crucible_shared SHARED "${CRUCIBLE_SHARED_STUB}")
    set_target_properties(crucible_shared PROPERTIES OUTPUT_NAME crucible VERSION ${PROJECT_VERSION}
                          SOVERSION 0 C_VISIBILITY_PRESET hidden CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
    if(APPLE)
        target_link_options(crucible_shared PRIVATE "LINKER:-force_load,$<TARGET_FILE:crucible_engine>")
    elseif(MSVC)
        target_link_options(crucible_shared PRIVATE "/WHOLEARCHIVE:$<TARGET_FILE:crucible_engine>")
    else()
        target_link_options(crucible_shared PRIVATE "LINKER:--whole-archive,$<TARGET_FILE:crucible_engine>,--no-whole-archive"
                            "LINKER:--exclude-libs,ALL")
    endif()
    target_link_libraries(crucible_shared PRIVATE crucible_engine)
    add_dependencies(crucible_shared crucible_engine)
endif()

if(CRUCIBLE_ENGINE_INSTALL)
    include(CMakePackageConfigHelpers)
    set(CRUCIBLE_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/CrucibleEngine")

    install(TARGETS crucible_engine EXPORT CrucibleEngineTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    install(DIRECTORY include/crucible DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" FILES_MATCHING PATTERN "*.h")
    install(FILES "${CRUCIBLE_GENERATED_DIR}/crucible/Version.h" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/crucible")
    if(TARGET crucible-render)
        install(TARGETS crucible-render crucible-measure crucible-module-check RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    endif()
    install(EXPORT CrucibleEngineTargets NAMESPACE crucible:: DESTINATION "${CRUCIBLE_INSTALL_CMAKEDIR}")

    configure_package_config_file(cmake/CrucibleEngineConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/CrucibleEngineConfig.cmake" INSTALL_DESTINATION "${CRUCIBLE_INSTALL_CMAKEDIR}")
    write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/CrucibleEngineConfigVersion.cmake"
        VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/CrucibleEngineConfig.cmake"
                  "${CMAKE_CURRENT_BINARY_DIR}/CrucibleEngineConfigVersion.cmake"
            DESTINATION "${CRUCIBLE_INSTALL_CMAKEDIR}")
    install(FILES LICENSE THIRD_PARTY.md DESTINATION "${CMAKE_INSTALL_DOCDIR}")

    # An installed copy must be usable through find_package() (needs a POSIX shell).
    if(CRUCIBLE_ENGINE_BUILD_TESTS AND NOT WIN32)
        add_test(NAME crucible_consumer
            COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/tools/test-consumer.sh" "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endif()
