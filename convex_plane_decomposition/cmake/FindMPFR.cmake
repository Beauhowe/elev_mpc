find_path(MPFR_INCLUDE_DIR
  NAMES mpfr.h
)

find_library(MPFR_LIBRARY
  NAMES mpfr
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MPFR
  REQUIRED_VARS MPFR_LIBRARY MPFR_INCLUDE_DIR
)

set(MPFR_LIBRARIES ${MPFR_LIBRARY})
set(MPFR_INCLUDE_DIRS ${MPFR_INCLUDE_DIR})

mark_as_advanced(MPFR_INCLUDE_DIR MPFR_LIBRARY)
