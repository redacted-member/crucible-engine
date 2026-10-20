# SPDX-License-Identifier: MIT
# Runs a command-line tool with a non-ASCII output path and checks that the file appears. On Windows main() receives
# argv in the ANSI code page, so this fails unless the tool converts its arguments to UTF-8 (tools/WinArgv.h) and opens
# files through crucible::utf8Path.
#   cmake -DTOOL=<crucible-render> -DWORK=<scratch dir> -P RenderUnicodePath.cmake
if(NOT TOOL OR NOT WORK)
    message(FATAL_ERROR "usage: cmake -DTOOL=<exe> -DWORK=<dir> -P RenderUnicodePath.cmake")
endif()

set(dir "${WORK}/Ünal_日本")
file(REMOVE_RECURSE "${dir}")
file(MAKE_DIRECTORY "${dir}")
if(NOT IS_DIRECTORY "${dir}")
    message(FATAL_ERROR "could not create ${dir}")
endif()

set(wav "${dir}/Ünal_日本.wav")
set(patch "${dir}/Ünal_日本.patch.json")
execute_process(COMMAND "${TOOL}" --length 0.25 --tail 0.1 --save-patch "${patch}" -o "${wav}"
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "crucible-render exited with ${rc}\n${out}\n${err}")
endif()
foreach(f "${wav}" "${patch}")
    if(NOT EXISTS "${f}")
        message(FATAL_ERROR "missing output ${f}\n${out}\n${err}")
    endif()
    file(SIZE "${f}" bytes)
    if(bytes LESS 100)
        message(FATAL_ERROR "${f} is only ${bytes} bytes")
    endif()
endforeach()

# Reading back through a non-ASCII path: load the patch we just wrote.
set(wav2 "${dir}/second_Ünal.wav")
execute_process(COMMAND "${TOOL}" --patch "${patch}" --length 0.1 --tail 0 -o "${wav2}"
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 0 OR NOT EXISTS "${wav2}")
    message(FATAL_ERROR "reading ${patch} failed (${rc})\n${out}\n${err}")
endif()
file(REMOVE_RECURSE "${dir}")
