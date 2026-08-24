foreach(required_variable SOURCE_MSG INSTALL_MSG EXPECTED_HASH)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} was not provided")
  endif()
endforeach()

foreach(message_file SOURCE_MSG INSTALL_MSG)
  if(NOT EXISTS "${${message_file}}")
    message(FATAL_ERROR "LiftStatus interface file is missing: ${${message_file}}")
  endif()
  file(SHA256 "${${message_file}}" actual_hash)
  if(NOT actual_hash STREQUAL EXPECTED_HASH)
    message(FATAL_ERROR
      "LiftStatus interface hash mismatch for ${${message_file}}: "
      "expected ${EXPECTED_HASH}, got ${actual_hash}")
  endif()
endforeach()
