
if(CONFIG_UART3_DMA OR CONFIG_UART2_DMA)

  message("UMDOM: modul UART_DMA Запуск сборки.")

  zephyr_include_directories(
    src/
  )

  zephyr_library()
  zephyr_library_compile_definitions(NO_POSIX_CHEATS)

  include(../stm32f1xx_ll.cmake)
  add_stm32_ll(usart gpio rcc)
  
  if(CONFIG_UART3_DMA)
  zephyr_library_sources(src/usart3_dma1_ch2_3.c)
  endif()

  if(CONFIG_UART3_DMA)
  zephyr_library_sources(src/usart2_dma1_ch6_7.c)
  endif()

endif()
