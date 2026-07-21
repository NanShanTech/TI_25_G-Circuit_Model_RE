# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal")
  file(MAKE_DIRECTORY "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal")
endif()
file(MAKE_DIRECTORY
  "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/1"
  "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal"
  "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal/tmp"
  "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal/src/h743_hal+h743_hal-stamp"
  "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal/src"
  "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal/src/h743_hal+h743_hal-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal/src/h743_hal+h743_hal-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/My_Git/2025-G-RE/2025-G/MDK-ARM/tmp/h743_hal+h743_hal/src/h743_hal+h743_hal-stamp${cfgdir}") # cfgdir has leading slash
endif()
