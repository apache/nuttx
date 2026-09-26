if(CONFIG_NET_LWIP)
  set(LWIP_BASE ${CMAKE_CURRENT_LIST_DIR}/lwip-2.0.3/src)

  file(GLOB LWIP_CORE_SRCS
       ${LWIP_BASE}/core/*.c
       ${LWIP_BASE}/core/ipv4/*.c
       ${LWIP_BASE}/api/*.c
       ${LWIP_BASE}/netif/*.c)

  if(CONFIG_NET_LWIP_IPV6)
    file(GLOB LWIP_IPV6_SRCS ${LWIP_BASE}/core/ipv6/*.c)
    list(APPEND LWIP_CORE_SRCS ${LWIP_IPV6_SRCS})
  endif()

  target_sources(net PRIVATE
    ${LWIP_CORE_SRCS}
    ${CMAKE_CURRENT_LIST_DIR}/lwip_init.c
    ${CMAKE_CURRENT_LIST_DIR}/sys_arch_nuttx.c
    ${CMAKE_CURRENT_LIST_DIR}/ethernetif_nuttx.c
    $<$<AND:$<BOOL:${CONFIG_ARCH_BOARD_SIM}>,$<BOOL:${CONFIG_NET_PKT}>>:${CMAKE_CURRENT_LIST_DIR}/lwip_pktif_sim.c>)

  target_include_directories(net PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lwip-2.0.3/src/include)
endif()
