/*
 * tcharx.h
 *
 * The generic-text names of <tchar.h> for the multibyte build of the Linux port. Dynomotion's
 * shared sources are written with TCHAR, _T("..."), _tcslen() and so on, which a Windows Unicode
 * build turns into wide characters. Here they are plain char and the C library.
 */
#ifndef TCHARX_H_
#define TCHARX_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>

typedef char TCHAR;
typedef char *LPTSTR;
typedef const char *LPCTSTR;
typedef char *LPSTR;
typedef const char *LPCSTR;

#define _T(x) x
#define _TEXT(x) x
#define _TRUNCATE ((size_t)-1)

#define _tcslen strlen
#define _tcscpy strcpy
#define _tcsncpy strncpy
#define _tcscat strcat
#define _tcscmp strcmp
#define _tcsncmp strncmp
#define _tcsicmp strcasecmp
#define _tcsnicmp strncasecmp
#define _tcschr strchr
#define _tcsrchr strrchr
#define _tcsstr strstr
#define _tstof atof
#define _ttoi atoi
#define _tfopen fopen
#define _trename rename
#define _tremove remove
#define _taccess access
#define _ftprintf fprintf
#define _tprintf printf
#define _sntprintf snprintf
#define _fgetts fgets
#define _fputts fputs
#define _ftscanf fscanf
#define _stscanf sscanf
#define _tmain main
#define wsprintf sprintf

// The _s variants truncate instead of failing, like their Windows versions with _TRUNCATE
#define _sntprintf_s(buf, size, count, ...) snprintf(buf, size, __VA_ARGS__)
#define _stprintf_p(buf, size, ...) snprintf(buf, size, __VA_ARGS__)

inline int _tcscpy_s(char *dst, size_t n, const char *src)
{
	if (n == 0) return 1;
	strncpy(dst, src, n);
	dst[n - 1] = '\0';
	return 0;
}

// fopen_s; the ",ccs=..." encoding suffix MSVC accepts in the mode is dropped
inline int _tfopen_s(FILE **f, const char *name, const char *mode)
{
	char m[16];
	size_t i = 0;
	for (const char *p = mode; *p && *p != ',' && i < sizeof(m) - 1; p++) m[i++] = *p;
	m[i] = '\0';
	*f = fopen(name, m);
	return *f ? 0 : errno;
}

inline char *_tfullpath(char *abs, const char *rel, size_t max)
{
	char tmp[PATH_MAX];
	if (realpath(rel, tmp) == NULL) return NULL;
	strncpy(abs, tmp, max);
	if (max) abs[max - 1] = '\0';
	return abs;
}

#endif /* TCHARX_H_ */
