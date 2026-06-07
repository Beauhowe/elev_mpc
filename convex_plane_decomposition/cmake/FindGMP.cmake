find_path(GMP_INCLUDE_DIR
  NAMES gmp.h
)

find_library(GMP_LIBRARY
  NAMES gmp
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GMP
  REQUIRED_VARS GMP_LIBRARY GMP_INCLUDE_DIR
)

set(GMP_LIBRARIES ${GMP_LIBRARY})
set(GMP_INCLUDE_DIRS ${GMP_INCLUDE_DIR})

mark_as_advanced(GMP_INCLUDE_DIR GMP_LIBRARY)
