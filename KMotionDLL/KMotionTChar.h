// KMotionTChar.h - the character type of the KMotion libraries' interfaces.
//
// KMotionDLL, GCodeInterpreter and KMotionServer are built as Unicode on Windows: every
// string in their interfaces is wchar_t. The Linux port (KMotionX) builds them as
// multibyte (UTF-8): char. Their own sources say TCHAR, which is that type in their
// build, but TCHAR follows the character set of whichever project includes a header,
// so a MultiByte project would read a wchar_t interface as char and neither compile
// nor link against the DLL. The public headers therefore say KMTCHAR: the type the
// libraries were built with, whatever the setting of the project including them.
// KMTEXT("...") is a string literal of that type.

#ifndef KMOTIONTCHAR_H
#define KMOTIONTCHAR_H

#ifdef _KMOTIONX
typedef char KMTCHAR;
#define KMTEXT(s) s
#else
typedef wchar_t KMTCHAR;
#define KMTEXT(s) L##s
#endif

#endif
