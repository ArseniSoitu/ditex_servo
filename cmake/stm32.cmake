include (CMakeForceCompiler)
#SET(CROSS_COMPILE "/home/soytu/gcc-arm-none-eabi-5_4-2016q3/bin/arm-none-eabi-")
SET(CROSS_COMPILE "arm-none-eabi-")

SET(FLAGS_CPU
		"-fno-strict-aliasing"
		"-fno-strength-reduce"
		"-fomit-frame-pointer"
		"-fmessage-length=0"
		"-nostartfiles"
		"-fno-exceptions"
		"-fno-rtti"
		"-ffunction-sections"
		"-fno-use-cxa-atexit"
		"-fdata-sections"
		"-mthumb"
		"-mcpu=cortex-m3"
)

SET(CMAKE_AR "${CROSS_COMPILE}ar")
SET(CC_OBJCOPY "${CROSS_COMPILE}objcopy")

CMAKE_FORCE_C_COMPILER("${CROSS_COMPILE}gcc" GNU)
CMAKE_FORCE_CXX_COMPILER("${CROSS_COMPILE}g++" GNU)

