# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/diego/esp/esp-idf-v4.4.5/components/bootloader/subproject"
  "/home/diego/proyectos/_/viejo/cr_c/build/bootloader"
  "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix"
  "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix/tmp"
  "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix/src/bootloader-stamp"
  "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix/src"
  "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/diego/proyectos/_/viejo/cr_c/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
