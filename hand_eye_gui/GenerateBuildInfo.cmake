if(NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_FILE)
    message(FATAL_ERROR "SOURCE_DIR and OUTPUT_FILE are required")
endif()

if(NOT DEFINED APP_BUILD_TYPE OR APP_BUILD_TYPE STREQUAL "")
    set(APP_BUILD_TYPE "unknown")
endif()

if(NOT DEFINED APP_CAMERA_BACKEND OR APP_CAMERA_BACKEND STREQUAL "")
    set(APP_CAMERA_BACKEND "unknown")
endif()
string(TOLOWER "${APP_CAMERA_BACKEND}" APP_CAMERA_BACKEND)
if(APP_CAMERA_BACKEND STREQUAL "orb")
    set(APP_CAMERA_BACKEND "orbbec")
endif()

if(NOT DEFINED APP_VERSION OR APP_VERSION STREQUAL "")
    set(APP_VERSION "unknown")
endif()

string(TIMESTAMP APP_BUILD_TIMESTAMP "%Y-%m-%dT%H:%M:%SZ" UTC)
execute_process(
    COMMAND git rev-parse --short=12 HEAD
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE GIT_COMMIT_RESULT
    OUTPUT_VARIABLE APP_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_COMMIT_RESULT EQUAL 0 OR APP_GIT_COMMIT STREQUAL "")
    set(APP_GIT_COMMIT "unknown")
endif()

# 包括未跟踪文件，避免实际参与打包的新文件仍被标记成 clean。
execute_process(
    COMMAND git status --porcelain --untracked-files=normal
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE GIT_STATUS_RESULT
    OUTPUT_VARIABLE APP_GIT_STATUS_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_STATUS_RESULT EQUAL 0)
    set(APP_GIT_WORKTREE_STATE "unknown")
elseif(APP_GIT_STATUS_OUTPUT STREQUAL "")
    set(APP_GIT_WORKTREE_STATE "clean")
else()
    set(APP_GIT_WORKTREE_STATE "dirty")
endif()

function(escape_cpp_string input output)
    string(REPLACE "\\" "\\\\" escaped "${input}")
    string(REPLACE "\"" "\\\"" escaped "${escaped}")
    string(REPLACE "\r" "\\r" escaped "${escaped}")
    string(REPLACE "\n" "\\n" escaped "${escaped}")
    set(${output} "${escaped}" PARENT_SCOPE)
endfunction()

escape_cpp_string("${APP_BUILD_TYPE}" APP_BUILD_TYPE_ESCAPED)
escape_cpp_string("${APP_BUILD_TIMESTAMP}" APP_BUILD_TIMESTAMP_ESCAPED)
escape_cpp_string("${APP_GIT_COMMIT}" APP_GIT_COMMIT_ESCAPED)
escape_cpp_string("${APP_GIT_WORKTREE_STATE}" APP_GIT_WORKTREE_STATE_ESCAPED)
escape_cpp_string("${APP_CAMERA_BACKEND}" APP_CAMERA_BACKEND_ESCAPED)
escape_cpp_string("${APP_VERSION}" APP_VERSION_ESCAPED)

get_filename_component(OUTPUT_DIR "${OUTPUT_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${OUTPUT_DIR}")
file(WRITE "${OUTPUT_FILE}"
"#pragma once\n"
"#define APP_BUILD_TYPE \"${APP_BUILD_TYPE_ESCAPED}\"\n"
"#define APP_BUILD_TIMESTAMP \"${APP_BUILD_TIMESTAMP_ESCAPED}\"\n"
"#define APP_GIT_COMMIT \"${APP_GIT_COMMIT_ESCAPED}\"\n"
"#define APP_GIT_WORKTREE_STATE \"${APP_GIT_WORKTREE_STATE_ESCAPED}\"\n"
"#define APP_CAMERA_BACKEND \"${APP_CAMERA_BACKEND_ESCAPED}\"\n"
"#define APP_VERSION \"${APP_VERSION_ESCAPED}\"\n")
