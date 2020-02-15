
# SPDX-License-Identifier: Apache-2.0

# CMake file to compile this Modbus component as a west module in Zephyr

#if (NOT DEFINED ENV{BSIM_COMPONENTS_PATH})
#  message(FATAL_ERROR "This Zephyr module requires the BabbleSim simulator.\
#Please set the  environment variable BSIM_COMPONENTS_PATH to point to its\
#components folder. More information can be found in\
#https://babblesim.github.io/folder_structure_and_env.html")
#endif()
#if (NOT DEFINED ENV{BSIM_OUT_PATH})
#  message(FATAL_ERROR "This Zephyr module requires the BabbleSim simulator.\
#Please set the  environment variable BSIM_OUT_PATH to point to the folder\
#where the simulator is compiled to. More information can be found in\
#https://babblesim.github.io/folder_structure_and_env.html")
#endif()

  message("UMDOM: modul UART3_DMA Запуск сборки.")

  zephyr_include_directories(
    src/
  )

  zephyr_library()
  zephyr_library_compile_definitions(NO_POSIX_CHEATS)

  set(FULL_LL_PATH $ENV{ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32f1xx/drivers/src)

  FILE(GLOB stm32_hal_sources 
    ${FULL_LL_PATH}/stm32f1xx_ll_usart.c
    ${FULL_LL_PATH}/stm32f1xx_ll_gpio.c
    ${FULL_LL_PATH}/stm32f1xx_ll_rcc.c)

  file(GLOB_RECURSE UART2_DMA_SRCS . src/*.c)

  zephyr_library_sources(${UART2_DMA_SRCS} ${stm32_hal_sources})
