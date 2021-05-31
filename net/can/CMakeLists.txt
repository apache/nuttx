if (CONFIG_NET_CAN)
  set(SRCS)

  # Socket layer
  list(APPEND SRCS can_sockif.c can_sendmsg.c can_recvmsg.c)

  if (CONFIG_NET_CAN_NOTIFIER)
    list(APPEND SRCS can_notifier.c)
  endif()

  if (CONFIG_NET_CANPROTO_OPTIONS)
    list(APPEND SRCS can_setsockopt.c can_getsockopt.c)
  endif()

  list(APPEND SRCS can_conn.c can_input.c can_callback.c can_poll.c)

  target_sources(net PRIVATE ${SRCS})
endif()
