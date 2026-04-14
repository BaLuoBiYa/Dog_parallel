#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qwen3tts::qwen3tts" for configuration "Debug"
set_property(TARGET qwen3tts::qwen3tts APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qwen3tts::qwen3tts PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libqwen3tts.so.0.1.0"
  IMPORTED_SONAME_DEBUG "libqwen3tts.so.0"
  )

list(APPEND _cmake_import_check_targets qwen3tts::qwen3tts )
list(APPEND _cmake_import_check_files_for_qwen3tts::qwen3tts "${_IMPORT_PREFIX}/lib/libqwen3tts.so.0.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
