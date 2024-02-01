// KMotionDLL.h    --  KMotion DLL Interface class
/*********************************************************************/
/*         Copyright (c) 2003-2006  DynoMotion Incorporated          */
/*********************************************************************/



#if !defined (KMOTIONDLL_H)
#define KMOTIONDLL_H



#include <afxmt.h>
#include "..\dsp_kogna\pc-dsp.h"

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the KMOTIONDLL_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// KMOTIONDLL_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef KMOTIONDLL_EXPORTS
#define KMOTIONDLL_API __declspec(dllexport)
#else
#define KMOTIONDLL_API __declspec(dllimport)
#endif
#pragma warning ( disable : 4251 )


#define MAX_LINE 2560

#define MAX_BOARDS 16

#define COMPILERTI_KFLOP "\\DSP_KFLOP\\c6000_7.4.24\\bin\\cl6x.exe"
#define LINKTEMPLATE_KFLOP "\\DSP_KFLOP\\LinkTemplate.cmd"
#define SYMBOLS_KFLOP "\\DSP_KFLOP\\DSPKFLOP.sym"
#define TILINKCMD_KFLOP "\\DSP_KFLOP\\Link.cmd"

#define COMPILERTI_KOGNA "\\DSP_KOGNA\\ccsv7\\tools\\compiler\\c6000_7.4.21\\bin\\cl6x.exe"
#define LINKTEMPLATE_KOGNA "\\DSP_KOGNA\\LinkTemplate.cmd"
#define SYMBOLS_KOGNA "\\DSP_KOGNA\\DSPKOGNA.sym"
#define TILINKCMD_KOGNA "\\DSP_KOGNA\\Link.cmd"

#define COMPILER "\\TCC67.exe"
#define VALIDATOR "\\DSP_KFLOP\\Cppcheck\\Cppcheck.exe"
#define VALIDATOR_SUPPRESS "\\DSP_KFLOP\\ValidateOptions.txt"
#define VALIDATEOPTIONS "\\DSP_KFLOP\\ValidateOptions.txt"


enum 
{
    KMOTION_OK=0,
    KMOTION_TIMEOUT=1,
    KMOTION_READY=2,
    KMOTION_ERROR=3,
};

enum 
{
    BOARD_TYPE_UNKNOWN=0,
    BOARD_TYPE_KFLOP=2,
    BOARD_TYPE_KOGNA=3,
};

enum // KMotionLocked Return Codes
{
	KMOTION_LOCKED=0,        // (and token is locked) if KMotion is available for use
	KMOTION_IN_USE=1,        // if already in use
	KMOTION_NOT_CONNECTED=2, // if error or not able to connect
};

typedef int CONSOLE_HANDLER(const char* buf);
typedef int CONSOLE_HANDLER_WIDE(const wchar_t* buf);
typedef void ERRMSG_HANDLER(const char* ErrMsg);
typedef void ERRMSG_HANDLER_WIDE(const wchar_t* ErrMsg);




// This class is exported from the KMotionDLL.dll
class KMOTIONDLL_API CKMotionDLL {
public:
	CKMotionDLL(int boardid);
	virtual ~CKMotionDLL();
    int BoardID;


	int WriteLineReadLine(const char *s, char *response);
	int WriteLine(const char *s);
	int WriteLineWithEcho(const char *s);
	int ReadLineTimeOut(char *buf, int TimeOutms=1000000);
	int ListLocations(int *nlocations, int *list);
	int WaitToken(char* CallerID);
	int WaitToken(bool display_msg=true, int TimeOut_ms=1000000, char *CallerID=NULL);
	int KMotionLock(char *CallerID=NULL);
	int USBLocation();
	int KMotionLockRecovery();
	void ReleaseToken();
	int Failed();
	int Disconnect();
	int FirmwareVersion();
	int CheckForReady();


	// Note: ALL User Thread Numbers start with 1
	
	
	int LoadCoff(int Thread, const char *Name, int PackToFlash=0); //PackToFlash 0=normal,1=NewVersion,2=bootload
	int CompileAndLoadCoff(const char *Name, int Thread);
	int CompileAndLoadCoff(const char* Name, int Thread, char* Err, int MaxErrLen);
	int CompileAndLoadCoff(const char* Name, int Thread, wchar_t *Err, int MaxErrLen);
	int Compile(const char* Name, const char* OutFile, const int board_type, int Thread, wchar_t *Err, int MaxErrLen);
	int Compile(const char* Name, const char* OutFile, const int board_type, int Thread, char *Err, int MaxErrLen);
	int CompileTI(const char * Name, const char * OutFile, const int BoardType, int Thread,wchar_t *Err, int MaxErrLen);
	int LinkTI(const char * Linker, const char * Name, const char * OutFile, const int BoardType, int Thread,wchar_t *Err, int MaxErrLen, int MaxSize);
	int ValidateC(const char *Name, wchar_t *Err, int MaxErrLen);
	int CheckCoffSize(const char *Name, int *size_text, int *size_bss, int *size_data, int *size_total);  // return size of sections and total (including padding)
	int CheckElfSize(const char* InFile, int* size_text, int* size_bss, int* size_data, int* size_total);
	unsigned int GetLoadAddress(int thread, int BoardType);
	void ConvertToOut(int thread, const char *InFile, char *OutFile, int MaxLength);
	void RemoveComments(CString &s);
	int ServiceConsole();

	int SetConsoleCallback(CONSOLE_HANDLER *ch);
	int SetErrMsgCallback(ERRMSG_HANDLER* eh);
	int SetErrMsgCallbackW(ERRMSG_HANDLER_WIDE* eh);
	int CheckKMotionVersion(int *type=NULL, bool GetBoardTypeOnly=false, bool Wait=true);
	int ExtractCoffVersionString(const char *InFile, char *Version);
	int ExtractElfVersionString(const char *InFile, char *Version);
	int ElfLoad(const char *InFile, unsigned int *EntryPoint, int PackToFlash);

	void DoErrMsg(const wchar_t* s);

	int GetStatus(MAIN_STATUS& status, bool lock);
	int FlashKognaCOM(const char* Com);
	CStringW Translate(CString s);
	void DoErrMsg(const char *s);

	bool ErrMessageDisplayed;

private:

	CMutex *PipeMutex;
	bool PipeOpen;
	bool ServerMessDisplayed;

	bool ReadStatus;

	CONSOLE_HANDLER* ConsoleHandler;
	CONSOLE_HANDLER_WIDE* ConsoleHandlerW;
	ERRMSG_HANDLER* ErrMsgHandler;
	ERRMSG_HANDLER_WIDE* ErrMsgHandlerW;

	int PipeCmd(int code);
	int PipeCmdStr(int code, const char *s);
	int Pipe(const char *s, int n, char *r, int *m);
	int LaunchServer();

	CString ExtractPath(CString InFile);

	CFile PipeFile;
};

#endif