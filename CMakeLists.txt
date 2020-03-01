
if(CONFIG_UART3_DMA)

  message("UMDOM: modul UART3_DMA Запуск сборки.")

  zephyr_include_directories(
    src/
  )

  zephyr_library()
  zephyr_library_compile_definitions(NO_POSIX_CHEATS)

  include(../stm32f1xx_ll.cmake)
  add_stm32_ll(usart gpio rcc)

  file(GLOB_RECURSE UART2_DMA_SRCS . src/*.c)

  zephyr_library_sources(${UART2_DMA_SRCS})

endif()
