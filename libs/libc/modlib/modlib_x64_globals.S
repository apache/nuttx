        .data
        .align 8
	.global globalNames

globalNames:
.L0:	.string	"accept4"
.L1:	.string	"clearenv"
.L2:	.string	"cmd_alias"
.L3:	.string	"cmd_arp"
.L4:	.string	"cmd_basename"
.L5:	.string	"cmd_break"
.L6:	.string	"cmd_cat"
.L7:	.string	"cmd_cd"
.L8:	.string	"cmd_cmp"
.L9:	.string	"cmd_cp"
.L10:	.string	"cmd_date"
.L11:	.string	"cmd_dd"
.L12:	.string	"cmd_dirname"
.L13:	.string	"cmd_dmesg"
.L14:	.string	"cmd_echo"
.L15:	.string	"cmd_exec"
.L16:	.string	"cmd_hexdump"
.L17:	.string	"cmd_insmod"
.L18:	.string	"cmd_kill"
.L19:	.string	"cmd_lbracket"
.L20:	.string	"cmd_losetup"
.L21:	.string	"cmd_ls"
.L22:	.string	"cmd_mkdir"
.L23:	.string	"cmd_mkfifo"
.L24:	.string	"cmd_mkrd"
.L25:	.string	"cmd_mount"
.L26:	.string	"cmd_mv"
.L27:	.string	"cmd_nslookup"
.L28:	.string	"cmd_poweroff"
.L29:	.string	"cmd_printf"
.L30:	.string	"cmd_pwd"
.L31:	.string	"cmd_rm"
.L32:	.string	"cmd_rmdir"
.L33:	.string	"cmd_rmmod"
.L34:	.string	"cmd_set"
.L35:	.string	"cmd_sleep"
.L36:	.string	"cmd_source"
.L37:	.string	"cmd_test"
.L38:	.string	"cmd_time"
.L39:	.string	"cmd_truncate"
.L40:	.string	"cmd_umount"
.L41:	.string	"cmd_unalias"
.L42:	.string	"cmd_uname"
.L43:	.string	"cmd_unset"
.L44:	.string	"cmd_uptime"
.L45:	.string	"cmd_usleep"
.L46:	.string	"cmd_xd"
.L47:	.string	"cmsg_append"
.L48:	.string	"dir_allocate"
.L49:	.string	"exec_builtin"
.L50:	.string	"icmp_alloc"
.L51:	.string	"icmp_chksum_iob"
.L52:	.string	"icmp_findconn"
.L53:	.string	"icmp_free"
.L54:	.string	"icmp_input"
.L55:	.string	"icmp_ioctl"
.L56:	.string	"icmp_nextconn"
.L57:	.string	"icmp_poll"
.L58:	.string	"icmp_pollsetup"
.L59:	.string	"icmp_pollteardown"
.L60:	.string	"icmp_recvmsg"
.L61:	.string	"icmp_reply"
.L62:	.string	"icmp_sendmsg"
.L63:	.string	"icmpv6_advertise"
.L64:	.string	"icmpv6_alloc"
.L65:	.string	"icmpv6_autoconfig"
.L66:	.string	"icmpv6_chksum"
.L67:	.string	"icmpv6_foreach"
.L68:	.string	"icmpv6_free"
.L69:	.string	"icmpv6_input"
.L70:	.string	"icmpv6_ioctl"
.L71:	.string	"icmpv6_linkipaddr"
.L72:	.string	"icmpv6_neighbor"
.L73:	.string	"icmpv6_nextconn"
.L74:	.string	"icmpv6_notify"
.L75:	.string	"icmpv6_poll"
.L76:	.string	"icmpv6_pollsetup"
.L77:	.string	"icmpv6_pollteardown"
.L78:	.string	"icmpv6_radvertise"
.L79:	.string	"icmpv6_recvmsg"
.L80:	.string	"icmpv6_reply"
.L81:	.string	"icmpv6_rnotify"
.L82:	.string	"icmpv6_rsolicit"
.L83:	.string	"icmpv6_rwait"
.L84:	.string	"icmpv6_rwait_cancel"
.L85:	.string	"icmpv6_rwait_setup"
.L86:	.string	"icmpv6_sendmsg"
.L87:	.string	"icmpv6_setaddresses"
.L88:	.string	"icmpv6_solicit"
.L89:	.string	"icmpv6_wait"
.L90:	.string	"icmpv6_wait_cancel"
.L91:	.string	"icmpv6_wait_setup"
.L92:	.string	"inet_close"
.L93:	.string	"insmod"
.L94:	.string	"ipv4_build_header"
.L95:	.string	"ipv4_chksum"
.L96:	.string	"ipv4_getpeername"
.L97:	.string	"ipv4_getsockname"
.L98:	.string	"ipv4_getsockopt"
.L99:	.string	"ipv4_input"
.L100:	.string	"ipv4_setsockopt"
.L101:	.string	"ipv4_upperlayer_chksum"
.L102:	.string	"ipv6_build_header"
.L103:	.string	"ipv6_getpeername"
.L104:	.string	"ipv6_getsockname"
.L105:	.string	"ipv6_getsockopt"
.L106:	.string	"ipv6_input"
.L107:	.string	"ipv6_setsockopt"
.L108:	.string	"ipv6_upperlayer_chksum"
.L109:	.string	"kill"
.L110:	.string	"kthread_create"
.L111:	.string	"loop_register"
.L112:	.string	"losetup"
.L113:	.string	"loteardown"
.L114:	.string	"lstat"
.L115:	.string	"mkrd"
.L116:	.string	"modhandle"
.L117:	.string	"modsym"
.L118:	.string	"mount"
.L119:	.string	"neighbor_add"
.L120:	.string	"neighbor_ethernet_out"
.L121:	.string	"neighbor_findentry"
.L122:	.string	"neighbor_lookup"
.L123:	.string	"neighbor_out"
.L124:	.string	"netdown_notifier_signal"
.L125:	.string	"oneshot_register"
.L126:	.string	"open_blockdriver"
.L127:	.string	"panic_notifier_call_chain"
.L128:	.string	"poll_notify"
.L129:	.string	"pthread_cancel"
.L130:	.string	"pthread_completejoin"
.L131:	.string	"pthread_createjoininfo"
.L132:	.string	"pthread_destroyjoin"
.L133:	.string	"pthread_detach"
.L134:	.string	"pthread_findjoininfo"
.L135:	.string	"pthread_mutex_inconsistent"
.L136:	.string	"pthread_release"
.L137:	.string	"pthread_sem_give"
.L138:	.string	"pthread_setup_scheduler"
.L139:	.string	"ramdisk_register"
.L140:	.string	"reboot_notifier_call_chain"
.L141:	.string	"recv"
.L142:	.string	"recvmsg"
.L143:	.string	"rmmod"
.L144:	.string	"romfs_checkmount"
.L145:	.string	"romfs_datastart"
.L146:	.string	"romfs_filecacheread"
.L147:	.string	"romfs_fileconfigure"
.L148:	.string	"romfs_finddirentry"
.L149:	.string	"romfs_freenode"
.L150:	.string	"romfs_fsconfigure"
.L151:	.string	"romfs_hwconfigure"
.L152:	.string	"romfs_hwread"
.L153:	.string	"romfs_img"
.L154:	.string	"romfs_img_len"
.L155:	.string	"romfs_parsedirentry"
.L156:	.string	"romfs_parsefilename"
.L157:	.string	"rtc_initialize"
.L158:	.string	"sem_clockwait"
.L159:	.string	"sem_destroy"
.L160:	.string	"sem_post"
.L161:	.string	"sem_timedwait"
.L162:	.string	"sem_trywait"
.L163:	.string	"sem_wait"
.L164:	.string	"send"
.L165:	.string	"setenv"
.L166:	.string	"sh_main"
.L167:	.string	"sigpending"
.L168:	.string	"sigprocmask"
.L169:	.string	"sigtimedwait"
.L170:	.string	"sotest_main"
.L171:	.string	"sync"
.L172:	.string	"sysinfo"
.L173:	.string	"tcpip_hdrsize"
.L174:	.string	"tls_init_info"
.L175:	.string	"unsetenv"
.L176:	.string	"waitpid"
.L177:	.string	"work_cancel"
.L178:	.string	"work_notifier_signal"
.L179:	.string	"work_queue"
.L180:	.string	"work_start_highpri"
.L181:	.string	"work_start_lowpri"
	.size	globalNames, . - globalNames

	.align	8
	.global	nGlobals
	.type	nGlobals, "object"
nGlobals:	
	.word	182
	.size	nGlobals, . - nGlobals

        .align 8
	.global globalTable
	.type	globalTable, "object"
globalTable:
	.weak	accept4
	.weak	clearenv
	.weak	cmd_alias
	.weak	cmd_arp
	.weak	cmd_basename
	.weak	cmd_break
	.weak	cmd_cat
	.weak	cmd_cd
	.weak	cmd_cmp
	.weak	cmd_cp
	.weak	cmd_date
	.weak	cmd_dd
	.weak	cmd_dirname
	.weak	cmd_dmesg
	.weak	cmd_echo
	.weak	cmd_exec
	.weak	cmd_hexdump
	.weak	cmd_insmod
	.weak	cmd_kill
	.weak	cmd_lbracket
	.weak	cmd_losetup
	.weak	cmd_ls
	.weak	cmd_mkdir
	.weak	cmd_mkfifo
	.weak	cmd_mkrd
	.weak	cmd_mount
	.weak	cmd_mv
	.weak	cmd_nslookup
	.weak	cmd_poweroff
	.weak	cmd_printf
	.weak	cmd_pwd
	.weak	cmd_rm
	.weak	cmd_rmdir
	.weak	cmd_rmmod
	.weak	cmd_set
	.weak	cmd_sleep
	.weak	cmd_source
	.weak	cmd_test
	.weak	cmd_time
	.weak	cmd_truncate
	.weak	cmd_umount
	.weak	cmd_unalias
	.weak	cmd_uname
	.weak	cmd_unset
	.weak	cmd_uptime
	.weak	cmd_usleep
	.weak	cmd_xd
	.weak	cmsg_append
	.weak	dir_allocate
	.weak	exec_builtin
	.weak	icmp_alloc
	.weak	icmp_chksum_iob
	.weak	icmp_findconn
	.weak	icmp_free
	.weak	icmp_input
	.weak	icmp_ioctl
	.weak	icmp_nextconn
	.weak	icmp_poll
	.weak	icmp_pollsetup
	.weak	icmp_pollteardown
	.weak	icmp_recvmsg
	.weak	icmp_reply
	.weak	icmp_sendmsg
	.weak	icmpv6_advertise
	.weak	icmpv6_alloc
	.weak	icmpv6_autoconfig
	.weak	icmpv6_chksum
	.weak	icmpv6_foreach
	.weak	icmpv6_free
	.weak	icmpv6_input
	.weak	icmpv6_ioctl
	.weak	icmpv6_linkipaddr
	.weak	icmpv6_neighbor
	.weak	icmpv6_nextconn
	.weak	icmpv6_notify
	.weak	icmpv6_poll
	.weak	icmpv6_pollsetup
	.weak	icmpv6_pollteardown
	.weak	icmpv6_radvertise
	.weak	icmpv6_recvmsg
	.weak	icmpv6_reply
	.weak	icmpv6_rnotify
	.weak	icmpv6_rsolicit
	.weak	icmpv6_rwait
	.weak	icmpv6_rwait_cancel
	.weak	icmpv6_rwait_setup
	.weak	icmpv6_sendmsg
	.weak	icmpv6_setaddresses
	.weak	icmpv6_solicit
	.weak	icmpv6_wait
	.weak	icmpv6_wait_cancel
	.weak	icmpv6_wait_setup
	.weak	inet_close
	.weak	insmod
	.weak	ipv4_build_header
	.weak	ipv4_chksum
	.weak	ipv4_getpeername
	.weak	ipv4_getsockname
	.weak	ipv4_getsockopt
	.weak	ipv4_input
	.weak	ipv4_setsockopt
	.weak	ipv4_upperlayer_chksum
	.weak	ipv6_build_header
	.weak	ipv6_getpeername
	.weak	ipv6_getsockname
	.weak	ipv6_getsockopt
	.weak	ipv6_input
	.weak	ipv6_setsockopt
	.weak	ipv6_upperlayer_chksum
	.weak	kill
	.weak	kthread_create
	.weak	loop_register
	.weak	losetup
	.weak	loteardown
	.weak	lstat
	.weak	mkrd
	.weak	modhandle
	.weak	modsym
	.weak	mount
	.weak	neighbor_add
	.weak	neighbor_ethernet_out
	.weak	neighbor_findentry
	.weak	neighbor_lookup
	.weak	neighbor_out
	.weak	netdown_notifier_signal
	.weak	oneshot_register
	.weak	open_blockdriver
	.weak	panic_notifier_call_chain
	.weak	poll_notify
	.weak	pthread_cancel
	.weak	pthread_completejoin
	.weak	pthread_createjoininfo
	.weak	pthread_destroyjoin
	.weak	pthread_detach
	.weak	pthread_findjoininfo
	.weak	pthread_mutex_inconsistent
	.weak	pthread_release
	.weak	pthread_sem_give
	.weak	pthread_setup_scheduler
	.weak	ramdisk_register
	.weak	reboot_notifier_call_chain
	.weak	recv
	.weak	recvmsg
	.weak	rmmod
	.weak	romfs_checkmount
	.weak	romfs_datastart
	.weak	romfs_filecacheread
	.weak	romfs_fileconfigure
	.weak	romfs_finddirentry
	.weak	romfs_freenode
	.weak	romfs_fsconfigure
	.weak	romfs_hwconfigure
	.weak	romfs_hwread
	.weak	romfs_img
	.weak	romfs_img_len
	.weak	romfs_parsedirentry
	.weak	romfs_parsefilename
	.weak	rtc_initialize
	.weak	sem_clockwait
	.weak	sem_destroy
	.weak	sem_post
	.weak	sem_timedwait
	.weak	sem_trywait
	.weak	sem_wait
	.weak	send
	.weak	setenv
	.weak	sh_main
	.weak	sigpending
	.weak	sigprocmask
	.weak	sigtimedwait
	.weak	sotest_main
	.weak	sync
	.weak	sysinfo
	.weak	tcpip_hdrsize
	.weak	tls_init_info
	.weak	unsetenv
	.weak	waitpid
	.weak	work_cancel
	.weak	work_notifier_signal
	.weak	work_queue
	.weak	work_start_highpri
	.weak	work_start_lowpri
	.quad	.L0, accept4
	.quad	.L1, clearenv
	.quad	.L2, cmd_alias
	.quad	.L3, cmd_arp
	.quad	.L4, cmd_basename
	.quad	.L5, cmd_break
	.quad	.L6, cmd_cat
	.quad	.L7, cmd_cd
	.quad	.L8, cmd_cmp
	.quad	.L9, cmd_cp
	.quad	.L10, cmd_date
	.quad	.L11, cmd_dd
	.quad	.L12, cmd_dirname
	.quad	.L13, cmd_dmesg
	.quad	.L14, cmd_echo
	.quad	.L15, cmd_exec
	.quad	.L16, cmd_hexdump
	.quad	.L17, cmd_insmod
	.quad	.L18, cmd_kill
	.quad	.L19, cmd_lbracket
	.quad	.L20, cmd_losetup
	.quad	.L21, cmd_ls
	.quad	.L22, cmd_mkdir
	.quad	.L23, cmd_mkfifo
	.quad	.L24, cmd_mkrd
	.quad	.L25, cmd_mount
	.quad	.L26, cmd_mv
	.quad	.L27, cmd_nslookup
	.quad	.L28, cmd_poweroff
	.quad	.L29, cmd_printf
	.quad	.L30, cmd_pwd
	.quad	.L31, cmd_rm
	.quad	.L32, cmd_rmdir
	.quad	.L33, cmd_rmmod
	.quad	.L34, cmd_set
	.quad	.L35, cmd_sleep
	.quad	.L36, cmd_source
	.quad	.L37, cmd_test
	.quad	.L38, cmd_time
	.quad	.L39, cmd_truncate
	.quad	.L40, cmd_umount
	.quad	.L41, cmd_unalias
	.quad	.L42, cmd_uname
	.quad	.L43, cmd_unset
	.quad	.L44, cmd_uptime
	.quad	.L45, cmd_usleep
	.quad	.L46, cmd_xd
	.quad	.L47, cmsg_append
	.quad	.L48, dir_allocate
	.quad	.L49, exec_builtin
	.quad	.L50, icmp_alloc
	.quad	.L51, icmp_chksum_iob
	.quad	.L52, icmp_findconn
	.quad	.L53, icmp_free
	.quad	.L54, icmp_input
	.quad	.L55, icmp_ioctl
	.quad	.L56, icmp_nextconn
	.quad	.L57, icmp_poll
	.quad	.L58, icmp_pollsetup
	.quad	.L59, icmp_pollteardown
	.quad	.L60, icmp_recvmsg
	.quad	.L61, icmp_reply
	.quad	.L62, icmp_sendmsg
	.quad	.L63, icmpv6_advertise
	.quad	.L64, icmpv6_alloc
	.quad	.L65, icmpv6_autoconfig
	.quad	.L66, icmpv6_chksum
	.quad	.L67, icmpv6_foreach
	.quad	.L68, icmpv6_free
	.quad	.L69, icmpv6_input
	.quad	.L70, icmpv6_ioctl
	.quad	.L71, icmpv6_linkipaddr
	.quad	.L72, icmpv6_neighbor
	.quad	.L73, icmpv6_nextconn
	.quad	.L74, icmpv6_notify
	.quad	.L75, icmpv6_poll
	.quad	.L76, icmpv6_pollsetup
	.quad	.L77, icmpv6_pollteardown
	.quad	.L78, icmpv6_radvertise
	.quad	.L79, icmpv6_recvmsg
	.quad	.L80, icmpv6_reply
	.quad	.L81, icmpv6_rnotify
	.quad	.L82, icmpv6_rsolicit
	.quad	.L83, icmpv6_rwait
	.quad	.L84, icmpv6_rwait_cancel
	.quad	.L85, icmpv6_rwait_setup
	.quad	.L86, icmpv6_sendmsg
	.quad	.L87, icmpv6_setaddresses
	.quad	.L88, icmpv6_solicit
	.quad	.L89, icmpv6_wait
	.quad	.L90, icmpv6_wait_cancel
	.quad	.L91, icmpv6_wait_setup
	.quad	.L92, inet_close
	.quad	.L93, insmod
	.quad	.L94, ipv4_build_header
	.quad	.L95, ipv4_chksum
	.quad	.L96, ipv4_getpeername
	.quad	.L97, ipv4_getsockname
	.quad	.L98, ipv4_getsockopt
	.quad	.L99, ipv4_input
	.quad	.L100, ipv4_setsockopt
	.quad	.L101, ipv4_upperlayer_chksum
	.quad	.L102, ipv6_build_header
	.quad	.L103, ipv6_getpeername
	.quad	.L104, ipv6_getsockname
	.quad	.L105, ipv6_getsockopt
	.quad	.L106, ipv6_input
	.quad	.L107, ipv6_setsockopt
	.quad	.L108, ipv6_upperlayer_chksum
	.quad	.L109, kill
	.quad	.L110, kthread_create
	.quad	.L111, loop_register
	.quad	.L112, losetup
	.quad	.L113, loteardown
	.quad	.L114, lstat
	.quad	.L115, mkrd
	.quad	.L116, modhandle
	.quad	.L117, modsym
	.quad	.L118, mount
	.quad	.L119, neighbor_add
	.quad	.L120, neighbor_ethernet_out
	.quad	.L121, neighbor_findentry
	.quad	.L122, neighbor_lookup
	.quad	.L123, neighbor_out
	.quad	.L124, netdown_notifier_signal
	.quad	.L125, oneshot_register
	.quad	.L126, open_blockdriver
	.quad	.L127, panic_notifier_call_chain
	.quad	.L128, poll_notify
	.quad	.L129, pthread_cancel
	.quad	.L130, pthread_completejoin
	.quad	.L131, pthread_createjoininfo
	.quad	.L132, pthread_destroyjoin
	.quad	.L133, pthread_detach
	.quad	.L134, pthread_findjoininfo
	.quad	.L135, pthread_mutex_inconsistent
	.quad	.L136, pthread_release
	.quad	.L137, pthread_sem_give
	.quad	.L138, pthread_setup_scheduler
	.quad	.L139, ramdisk_register
	.quad	.L140, reboot_notifier_call_chain
	.quad	.L141, recv
	.quad	.L142, recvmsg
	.quad	.L143, rmmod
	.quad	.L144, romfs_checkmount
	.quad	.L145, romfs_datastart
	.quad	.L146, romfs_filecacheread
	.quad	.L147, romfs_fileconfigure
	.quad	.L148, romfs_finddirentry
	.quad	.L149, romfs_freenode
	.quad	.L150, romfs_fsconfigure
	.quad	.L151, romfs_hwconfigure
	.quad	.L152, romfs_hwread
	.quad	.L153, romfs_img
	.quad	.L154, romfs_img_len
	.quad	.L155, romfs_parsedirentry
	.quad	.L156, romfs_parsefilename
	.quad	.L157, rtc_initialize
	.quad	.L158, sem_clockwait
	.quad	.L159, sem_destroy
	.quad	.L160, sem_post
	.quad	.L161, sem_timedwait
	.quad	.L162, sem_trywait
	.quad	.L163, sem_wait
	.quad	.L164, send
	.quad	.L165, setenv
	.quad	.L166, sh_main
	.quad	.L167, sigpending
	.quad	.L168, sigprocmask
	.quad	.L169, sigtimedwait
	.quad	.L170, sotest_main
	.quad	.L171, sync
	.quad	.L172, sysinfo
	.quad	.L173, tcpip_hdrsize
	.quad	.L174, tls_init_info
	.quad	.L175, unsetenv
	.quad	.L176, waitpid
	.quad	.L177, work_cancel
	.quad	.L178, work_notifier_signal
	.quad	.L179, work_queue
	.quad	.L180, work_start_highpri
	.quad	.L181, work_start_lowpri
	.size	globalTable, . - globalTable
