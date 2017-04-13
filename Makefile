#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := globecopter_led

COMPONENT_ADD_INCLUDEDIRS := port/include include

include $(IDF_PATH)/make/project.mk

