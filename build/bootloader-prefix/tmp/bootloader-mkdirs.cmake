# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/diego/esp/esp-idf/components/bootloader/subproject"
  "/home/diego/proyectos/cr_c/build/bootloader"
  "/home/diego/proyectos/cr_c/build/bootloader-prefix"
  "/home/diego/proyectos/cr_c/build/bootloader-prefix/tmp"
  "/home/diego/proyectos/cr_c/build/bootloader-prefix/src/bootloader-stamp"
  "/home/diego/proyectos/cr_c/build/bootloader-prefix/src"
  "/home/diego/proyectos/cr_c/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/diego/proyectos/cr_c/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/diego/proyectos/cr_c/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
