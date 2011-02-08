# Copyright (C) 2007 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# BoardConfig.mk
#
# Product-specific compile-time definitions.
#

# Set this up here so that BoardVendorConfig.mk can override it
BOARD_USES_GENERIC_AUDIO := false

#BOARD_USES_LIBSECRIL_STUB := true

#BOARD_NO_PAGE_FLIPPING := true

# Use the non-open-source parts, if they're present
#-include vendor/samsung/crespo/BoardConfigVendor.mk

TARGET_CPU_ABI := armeabi-v7a
TARGET_CPU_ABI2 := armeabi

BOARD_HAVE_BLUETOOTH := false
BOARD_HAVE_BLUETOOTH_BCM := false

TARGET_NO_BOOTLOADER := true

TARGET_NO_KERNEL := true

TARGET_NO_RADIOIMAGE := true
TARGET_PROVIDES_INIT_TARGET_RC := true
TARGET_BOARD_PLATFORM := omap3
TARGET_BOOTLOADER_BOARD_NAME := bc10

TARGET_SEC_INTERNAL_STORAGE := false

# Enable NEON feature
TARGET_ARCH_VARIANT := armv7-a-neon
ARCH_ARM_HAVE_TLS_REGISTER := true

ifeq ($(BOARD_USES_GENERIC_AUDIO),false)
BOARD_USES_ALSA_AUDIO := true
BUILD_WITH_ALSA_UTILS := true
endif

USE_CAMERA_STUB := true
ifeq ($(USE_CAMERA_STUB),false)
BOARD_CAMERA_LIBRARIES := libcamera
endif

BOARD_USES_HGL := false
BOARD_USES_OVERLAY := false

DEFAULT_FB_NUM := 2

#BOARD_NAND_PAGE_SIZE := 4096 -s 128

#BOARD_KERNEL_BASE := 0x30000000
#BOARD_KERNEL_PAGESIZE := 4096
#BOARD_KERNEL_CMDLINE := console=ttyFIQ0 no_console_suspend

#TARGET_RECOVERY_UI_LIB := librecovery_ui_crespo
TARGET_RELEASETOOLS_EXTENSIONS := device/beatcraft/bc10

TARGET_USERIMAGES_USE_EXT4 := false
#BOARD_SYSTEMIMAGE_PARTITION_SIZE := 536870912
#BOARD_USERDATAIMAGE_PARTITION_SIZE := 1073741824
#BOARD_FLASH_BLOCK_SIZE := 4096

# Connectivity - Wi-Fi
WPA_SUPPLICANT_VERSION := VER_0_6_X
BOARD_WPA_SUPPLICANT_DRIVER := AWEXT
BOARD_WLAN_DEVICE := rt3070
WIFI_DRIVER_MODULE_PATH     := "/system/lib/modules/rt3070sta.ko"
#WIFI_DRIVER_FW_STA_PATH     := "/vendor/firmware/fw_bcm4329.bin"
#WIFI_DRIVER_FW_AP_PATH      := "/vendor/firmware/fw_bcm4329_apsta.bin"
WIFI_DRIVER_MODULE_NAME     :=  "rt3070sta"
#WIFI_DRIVER_MODULE_ARG      :=  "firmware_path=/vendor/firmware/fw_bcm4329.bin nvram_path=/vendor/firmware/nvram_net.txt"

