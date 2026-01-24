cmake_minimum_required(VERSION 3.20)

project(target-USB_Library LANGUAGES C CXX)
 
set(SOURCES
	Class/CDC/Src/usbd_cdc.c
	Core/Src/usbd_core.c
	Core/Src/usbd_ctlreq.c
	Core/Src/usbd_ioreq.c
)

set(INCLUDES
    Class/CDC/Inc
    Core/Inc
)
 
add_library(USB_Library STATIC ${SOURCES})
target_include_directories(USB_Library PUBLIC ${INCLUDES})