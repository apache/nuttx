#ifndef _RGMP_STDARG_H
#define	_RGMP_STDARG_H

typedef char *va_list;

#define	__va_size(type) \
	(((sizeof(type) + sizeof(long) - 1) / sizeof(long)) * sizeof(long))

#ifndef va_start
#define	va_start(ap, last) \
	((ap) = (va_list)&(last) + __va_size(last))
#endif

#ifndef va_arg
#define	va_arg(ap, type) \
	(*(type *)((ap) += __va_size(type), (ap) - __va_size(type)))
#endif

#ifndef va_end
#define	va_end(ap)	((void)0)
#endif

#endif
