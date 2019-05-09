#!/bin/bash
rm tags
ctags -R firmware/include firmware/src chibios/os/ports/common chibios/os/ports/GCC chibios/os/hal/{include,src,platforms/STM32F4xx,platforms/STM32} common -o tags
export VIM_TAGS_FILE=`pwd`/tags

