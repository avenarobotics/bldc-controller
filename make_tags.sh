#!/bin/bash
rm tags
touch unsorted
find . -name "*" -not -path "./chibios/*" | xargs ctags -a unsorted
ctags -a unsorted -R chibios/ext
ctags -a unsorted -R chibios/os/hal/platforms/STM32
ctags -a unsorted -R chibios/os/hal/platforms/STM32F4xx
ctags -a unsorted -R chibios/os/hal/src
ctags -a unsorted -R chibios/os/hal/include
sort unsorted >> tags
rm unsorted
export VIM_TAGS_FILE=`pwd`/tags

