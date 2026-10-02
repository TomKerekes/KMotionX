// KMotionDLL.h    --  KMotion DLL Interface class
/*********************************************************************/
/*         Copyright (c) 2003-2006  DynoMotion Incorporated          */
/*********************************************************************/



#if !defined (KMOTIONDLL_H)
#define KMOTIONDLL_H



#include <afxmt.h>
#include <afxtempl.h> // Include this header for CList
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
#define VALIDATOR "\\DSP_KFLOP\\clang\\clang-tidy.exe"
#define VALIDATECHECKS "\\DSP_KFLOP\\clang\\.clang-tidy"
#define VALIDATEOPTIONS "\\DSP_KFLOP\\clang\\ValidateOptions.txt"


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

typedef int CONSOLE_HANDLER(const TCHAR* buf);
typedef void ERRMSG_HANDLER(const TCHAR* ErrMsg);




// This class is exported from the KMotionDLL.dll
class KMOTIONDLL_API CKMotionDLL {
public:
	CKMotionDLL(int boardid);
	virtual ~CKMotionDLL();
    int BoardID;

	bool NoEthernet = false; // set to true if not to attempt to use ethernet

	int WriteLineReadLine(const char *s, char *response);
	int WriteLine(const char *s);
	int WriteLineWithEcho(const char *s);
	int ReadLineTimeOut(char *buf, int TimeOutms=20000);
	int ListLocations(int *nlocations, int *list);
	int WaitToken(char* CallerID);
	int WaitToken(bool display_msg=true, int TimeOut_ms=20000, char *CallerID=NULL);
	int KMotionLock(char *CallerID=NULL);
	int USBLocation();
	int KMotionLockRecovery();
	void ReleaseToken();
	int Failed();
	int Disconnect();
	int FirmwareVersion();
	int CheckForReady();


	// Note: ALL User Thread Numbers start with 1
	
	
	int LoadCoff(int Thread, const TCHAR *Name, int PackToFlash=0); //PackToFlash 0=normal,1=NewVersion,2=bootload
#ifdef _UNICODE
	int LoadCoff(int Thread, const char* Name, int PackToFlash);
#endif //PackToFlash 0=normal,1=NewVersion,2=bootload
	int CompileAndLoadCoff(const TCHAR *Name, int Thread);
	int CompileAndLoadCoff(const TCHAR* Name, int Thread, TCHAR* Err, int MaxErrLen);
	int Compile(const TCHAR* Name, const TCHAR* OutFile, const int board_type, int Thread, TCHAR *Err, int MaxErrLen);
	int RemoveBOMandIncludedFiles(const CString& FilePath, const CList<CString, CString&>& IncludePaths);
	int CompileTI(const TCHAR * Name, const TCHAR * OutFile, const int BoardType, int Thread,TCHAR *Err, int MaxErrLen);
	int LinkTI(const TCHAR * Linker, const TCHAR * Name, const TCHAR * OutFile, const int BoardType, int Thread,TCHAR *Err, int MaxErrLen, int MaxSize);
	int ValidateC(const TCHAR *Name, TCHAR *Err, int MaxErrLen, int BoardType);
	CStringA W2UTF8(const TCHAR* pszText, int nLength);
	CString UTF82W(const char* pszText, int nLength);
	int CheckCoffSize(const TCHAR *Name, int *size_text, int *size_bss, int *size_data, int *size_total);  // return size of sections and total (including padding)
	int CheckElfSize(const TCHAR* InFile, int* size_text, int* size_bss, int* size_data, int* size_total);
	unsigned int GetLoadAddress(int thread, int BoardType);
	void ConvertToOut(int thread, const TCHAR *InFile, TCHAR *OutFile, int MaxLength);
	void RemoveComments(CString &s);
	int ConvertCRToCRLF(const CString filePath);
	int RemoveUTF8BOM(const CString filePath);  // remove UTF8 BOM from file
	int ServiceConsole();

	int SetConsoleCallback(CONSOLE_HANDLER *ch);
	int SetErrMsgCallback(ERRMSG_HANDLER* eh);
	int CheckKMotionVersion(int *type=NULL, bool GetBoardTypeOnly=false, bool Wait=true);
	int ExtractCoffVersionString(const TCHAR *InFile, char *Version);
	int ExtractElfVersionString(const TCHAR *InFile, char *Version);
	int ElfLoad(const TCHAR *InFile, unsigned int *EntryPoint, int PackToFlash);

	void DoErrMsg(const TCHAR* s);

	int GetStatus(MAIN_STATUS& status, bool lock);
#ifdef _UNICODE
	int FlashKognaCOM(const char* Com);
#endif
	int FlashKognaCOM(const TCHAR* Com);
	CString Translate(CString s);

	bool ErrMessageDisplayed;

private:

	CMutex *PipeMutex;
	bool PipeOpen;
	bool ServerMessDisplayed;

	bool ReadStatus;

	CONSOLE_HANDLER* ConsoleHandler;
	ERRMSG_HANDLER* ErrMsgHandler;

	int PipeCmd(int code);
	int PipeCmdStr(int code, const char *s);
	int Pipe(const char *s, int n, char *r, int *m);
	int LaunchServer();

	CString ExtractPath(CString InFile);

	CFile PipeFile;
};

#endif