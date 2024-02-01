#include "KMotionDLL.h"


// KMotionDLL.cpp : Defines the entry point for the DLL application.
/*********************************************************************/
/*         Copyright (c) 2003-2006  DynoMotion Incorporated          */
/*********************************************************************/


#include "stdafx.h"

#define KMotionBd 1

#include "PARAMS.h"
#include "VERSION.h"
#include "COFF.h"
#include "CLOAD.h"
#include "Elf.h"


extern CString MainPathDLL;
extern CString MainPathDLL64;
extern CString MainPath;
extern CString MainPathRoot;


CKMotionDLL::CKMotionDLL(int boardid)
{ 
	BoardID = boardid;
	PipeOpen = false;
	ServerMessDisplayed = false;
	ErrMessageDisplayed = false;
	ReadStatus = true;
	PipeMutex = new CMutex(FALSE, "KMotionPipe", NULL);

	ConsoleHandler = NULL;
	ErrMsgHandlerW = NULL;
	ErrMsgHandler = NULL;
}

CKMotionDLL::~CKMotionDLL()
{
	LPTSTR lpszPipename = "\\\\.\\pipe\\kmotionpipe"; 
	if (PipeOpen)
	{
		PipeOpen=false;
		if(share==2)
		{
			PipeFile.Close();
			Sleep(100);  // give some time for Server to close
		}
	}
	delete PipeMutex;
}


int CKMotionDLL::WriteLineReadLine(const char *s, char *response)
{

	// Send Code, board, string -- Get Dest (byte), Result (int), and string
	char d[MAX_LINE+1];
	char r[MAX_LINE+1];
	int result, m, code = ENUM_WriteLineReadLine;
	
	result = WaitToken("WriteLineReadLine");

	if (result) return result;

	memcpy(d,&code,4); 
	memcpy(d+4,&BoardID,4); 
	strcpy(d+8,s); 

	Pipe(d, (int)strlen(s)+1+8 ,r, &m);

	memcpy(&result,r+1,4);
	strcpy(response,r+1+4); 
	
	ReleaseToken();
	return result;
}

int CKMotionDLL::WriteLine(const char *s)
{
	return PipeCmdStr(ENUM_WriteLine, s);
}

int CKMotionDLL::WriteLineWithEcho(const char *s)
{
	return PipeCmdStr(ENUM_WriteLineWithEcho, s);
}

int CKMotionDLL::ReadLineTimeOut(char *response, int TimeOutms)
{
	// Send Code, BoardID, timeout -- Get Dest, Result (int), and string
	char d[MAX_LINE+1];
	char r[MAX_LINE+1];
	int result, m, code = ENUM_ReadLineTimeOut;

	memcpy(d,&code,4); 
	memcpy(d+4,&BoardID,4); 
	memcpy(d+8,&TimeOutms,4); 

	Pipe(d, 12 ,r, &m);

	memcpy(&result,r+1,4);
	strcpy(response,r+1+4); 
	
	return result;
}

int CKMotionDLL::ListLocations(int *nlocations, int *list)
{
	// Send Code -- Get Dest, Result (int), nlocations (int), List (ints)
	char d[MAX_LINE+1];
	char r[MAX_LINE+1];
	int result, m, code = ENUM_ListLocations;

	memcpy(d,&code,4); 

	Pipe(d, 4 ,r, &m);

	memcpy(&result,r+1,4);
	if (result==0) 
	{
		memcpy(nlocations,r+1+4,4); 
		memcpy(list,r+1+8, *nlocations*sizeof(int)); 
	}
	else
	{
		*nlocations=0; 
	}
	return result;
}

int CKMotionDLL::Failed()
{
	return PipeCmd(ENUM_Failed);
}

int CKMotionDLL::Disconnect()
{
	return PipeCmd(ENUM_Disconnect);
}

int CKMotionDLL::FirmwareVersion()
{
	return PipeCmd(ENUM_FirmwareVersion);
}

int CKMotionDLL::CheckForReady()
{
	return PipeCmd(ENUM_CheckForReady);
}

int CKMotionDLL::KMotionLock(char *CallerID)
{
	return PipeCmdStr(ENUM_KMotionLock, CallerID);
}

int CKMotionDLL::USBLocation()
{
	return PipeCmd(ENUM_USBLocation);
}

int CKMotionDLL::KMotionLockRecovery()
{
	return PipeCmd(ENUM_KMotionLockRecovery);
}

// Try and get the token for the Board
//
//    return with the token (return value = KMOTION_LOCKED)
// OR 
//    if there is a problem with the board 
//    display a message (return value = KMOTION_NOT_CONNECTED)


int CKMotionDLL::WaitToken(char* CallerID)
{
	return WaitToken(false, 100000, CallerID);
}

int CKMotionDLL::WaitToken(bool display_msg, int TimeOut_ms, char* CallerID)
{
	CHiResTimer Timer;
	int result;

//tktk
// 	if (ErrMessageDisplayed) 
//		return KMOTION_NOT_CONNECTED;

	Timer.Start();

	int count=0;
	do
	{
		// this Mutex helps maintain a waiting list
		// so everybody gets a chance at the token
		// rather than leaving it random.  Also make
		// sure we have everything before we proceed
		// so we don't get stuck somewhere (deadlocked)
		
		if (!PipeMutex->Lock(TimeOut_ms))
		{
			return KMOTION_IN_USE;
		}

		if (Timer.Elapsed_Seconds() > 2.0 * TimeOut_ms * 0.001)
		{
			PipeMutex->Unlock();
			return KMOTION_IN_USE;
		}

		if (count++)
		{
			Sleep(10);
		}

		result = KMotionLock(CallerID);

		if (result == KMOTION_IN_USE)
			PipeMutex->Unlock();
	}
	while (result == KMOTION_IN_USE);

	if (result == KMOTION_NOT_CONNECTED && display_msg)
	{
		char s[256];

		if ((unsigned int)BoardID > MAX_USB_ID)
			sprintf(s, " %u.%u.%u.%u", (BoardID>>24)&0xff, (BoardID >> 16) & 0xff, (BoardID >> 8) & 0xff, BoardID & 0xff);
		else if (BoardID>0)
			sprintf(s," 0x%X", BoardID);
		else
			sprintf(s," #%d", BoardID);

		DoErrMsg(Translate("Can't Connect to KMotion Board") + s);

	}

	if (result != KMOTION_LOCKED) 
		PipeMutex->Unlock();      // keep the pipe if we have the token
	
	return result;
}

void CKMotionDLL::ReleaseToken()
{
	PipeCmd(ENUM_ReleaseToken);
	PipeMutex->Unlock();      // also release the pipe
}

int  CKMotionDLL::ServiceConsole()
{
	return PipeCmd(ENUM_ServiceConsole);
}

int CKMotionDLL::SetConsoleCallback(CONSOLE_HANDLER *ch)
{
	ConsoleHandler = ch;

	// tell the server who is the server for 
	// the console 
	return PipeCmd(ENUM_SetConsole);
}

int CKMotionDLL::SetErrMsgCallback(ERRMSG_HANDLER *ch)
{
	ErrMsgHandler = ch;
	return 0;
}

int CKMotionDLL::SetErrMsgCallbackW(ERRMSG_HANDLER_WIDE* ch)
{
	ErrMsgHandlerW = ch;
	return 0;
}


// Note: ALL User Thread Numbers start with 1

int CKMotionDLL::LoadCoff(int Thread, const char *Name, int PackToFlash)
{
	CString s;
	unsigned int EntryPoint;

	if (Thread==0) return 1;

	if (PackToFlash==0 && CheckKMotionVersion()) return 1; 

	if (PackToFlash==0)
	{
		s.Format("Kill %d", Thread);  // make sure the Thread isn't running
		if (WriteLine(s)) return 1;
	}
	
	int result =  ::LoadCoff(this, Name, &EntryPoint, PackToFlash);

	// failed try as Elf File
	if (result) result = ElfLoad(Name, &EntryPoint, PackToFlash);

	if (result)
	{
		MessageBoxW(NULL, Translate("error loading file"), L"KMotion", MB_OK | MB_SYSTEMMODAL);
		return result;
	}

	if (Thread >= 0 && PackToFlash==0)
	{
		// Set the entry point for the thread
		
		s.Format("EntryPoint%d %X",Thread,EntryPoint);
		result = WriteLine(s);
		if (result) return result;
	}

	return 0;
}


// send code, board

int CKMotionDLL::PipeCmd(int code)
{
	char d[MAX_LINE+1];
	char r[MAX_LINE+1];
	int result, m;

	memcpy(d,&code,4); 
	memcpy(d+4,&BoardID,4); 

	Pipe(d, 8 ,r, &m);

	memcpy(&result,r+1,4);
	
	return result;
}

// send code, board, string

int CKMotionDLL::PipeCmdStr(int code, const char *s)
{
	char d[MAX_LINE+1];
	char r[MAX_LINE+1];
	int result, m;

	memcpy(d,&code,4); 
	memcpy(d+4,&BoardID,4); 

	if (s == NULL)
	{
		d[8] = 0;
		Pipe(d, 1 + 8, r, &m);
	}
	else
	{
		strcpy(d + 8, s);
		Pipe(d, (int)strlen(s) + 1 + 8, r, &m);
	}

	memcpy(&result,r+1,4);
	
	return result;
}




int CKMotionDLL::Pipe(const char *s, int n, char *r, int *m)
{
	unsigned char Reply = 0xAA;
	CString ErrorMsg;
	bool ReceivedErrMsg=false;


	static int EntryCount=0;

	if (ServerMessDisplayed) 
		return 1;

	LPTSTR lpszPipename = "\\\\.\\pipe\\kmotionpipe"; 

	try
	{
		PipeMutex->Lock();

		if (EntryCount>0)
		{
			int Result=KMOTION_IN_USE;
			memcpy(r+1,&Result,sizeof(Result));
			PipeMutex->Unlock();
			return 1;
		}
	
		EntryCount++;

		if (!PipeOpen)
		{
			int i;
			
			PipeOpen=true;  // only try once
			if (!PipeFile.Open(lpszPipename, CFile::modeReadWrite))
			{
				// pipe won't open try to launch server
				LaunchServer();
				
				for (i=0; i<100; i++) // try for a few secs
				{
					if (PipeFile.Open(lpszPipename, CFile::modeReadWrite))
						break;
					
					Sleep(100);
				}

				if (i==100)
				{
					EntryCount--;
					if (ServerMessDisplayed) return 1;
					ServerMessDisplayed=TRUE;
					DoErrMsg(Translate("Unable to Connect to KMotion Server"));
					PipeMutex->Unlock();
					exit(1);
				}
			}
		}

		PipeFile.Write(s, n);           // Send the request
		
		for (;;)
		{
			*m = PipeFile.Read(r, MAX_LINE+1);     // Get the response

			// the first byte of the response is the destination
			// currently DEST_NORMAL, DEST_CONSOLE
			
			if (*r == DEST_CONSOLE)
			{
				PipeFile.Write(&Reply, 1);     // Send an ACK back to server
		
				// send it to the console if someone registered a callback

				if (ConsoleHandler)
					ConsoleHandler(r+1);
			}
			else if (*r == DEST_ERRMSG)
			{
				PipeFile.Write(&Reply, 1);     // Send an ACK back to server
				
				// because callback might throw an exception, delay doing the User Callback
				// until everything is received back from the Server and we clean up
				
				ErrorMsg=r+1;
				ReceivedErrMsg=true;
			}
			else
			{
				break;
			}
		}

		EntryCount--;

		PipeMutex->Unlock();
	}
	catch (CFileException *e)
	{
		e->Delete();  // to avoid memory leak
		EntryCount--;
		if (ServerMessDisplayed) return 1;
		ServerMessDisplayed=TRUE;
		DoErrMsg(Translate("Unable to Connect to KMotion Server"));
		PipeMutex->Unlock();
		exit(1);
	}

	if (ReceivedErrMsg)
	{
		DoErrMsg(Trans.Translate(ErrorMsg));
	}
	
	return 0;
 }



int CKMotionDLL::LaunchServer()
{
	SECURITY_ATTRIBUTES sa          = {0};
	STARTUPINFO         si          = {0};
	PROCESS_INFORMATION pi          = {0};
	HANDLE              hPipeOutputRead  = NULL;
	HANDLE              hPipeOutputWrite = NULL;
	HANDLE              hPipeInputRead   = NULL;
	HANDLE              hPipeInputWrite  = NULL;
	BOOL                bTest = 0;
	DWORD               dwNumberOfBytesRead = 0;
	
	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;


	
	
	// Create pipe for standard output redirection.
	CreatePipe(&hPipeOutputRead,  // read handle
		&hPipeOutputWrite, // write handle
		&sa,      // security attributes
		0      // number of bytes reserved for pipe - 0 default
		);
	
	// Create pipe for standard input redirection.
	CreatePipe(&hPipeInputRead,  // read handle
		&hPipeInputWrite, // write handle
		&sa,      // security attributes
		0      // number of bytes reserved for pipe - 0 default
		);
	
	// Make child process use hPipeOutputWrite as standard out,
	// and make sure it does not show on screen.
	si.cb = sizeof(si);
	si.dwFlags     = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
	si.wShowWindow = SW_HIDE;
	si.hStdInput   = hPipeInputRead;
	si.hStdOutput  = hPipeOutputWrite;
	si.hStdError   = hPipeOutputWrite;

	CString cmd;  // build command line

	cmd = MainPathDLL64 + "\\KMotionServer.exe";
	
	if (!CreateProcess (
		NULL,
		cmd.GetBuffer(0), 
		NULL, NULL,
		TRUE, 0,
		NULL, NULL,
		&si, &pi))
	{
		ServerMessDisplayed = true;
		CStringW Errmsg;
		Errmsg.Format("Unable to execute:\r\r" + (CStringW)cmd + Translate("\r\rTry re-installing software or copy this file to the same location as KMotion.exe or Calling Application"));
		DoErrMsg(Errmsg);
		exit(1);
	}

	
	// Now that handles have been inherited, close it to be safe.
	// You don't want to read or write to them accidentally.
	CloseHandle(hPipeOutputWrite);
	CloseHandle(hPipeInputRead);

	return 0;
}

int CKMotionDLL::CompileAndLoadCoff(const char *Name, int Thread)
{
	return CompileAndLoadCoff(Name, Thread, (char *)NULL, 0);
}

int CKMotionDLL::CompileAndLoadCoff(const char* Name, int Thread, char *Err, int MaxErrLen)
{
	wchar_t* ErrW = new wchar_t[max(MaxErrLen,1)];
	*ErrW = 0;
	int rslt = CompileAndLoadCoff(Name, Thread, ErrW, MaxErrLen);
	if (Err != NULL) strcpy_s(Err, MaxErrLen, CW2A(ErrW));
	delete ErrW;
	return rslt;
}

int CKMotionDLL::CompileAndLoadCoff(const char* Name, int Thread, wchar_t* Err, int MaxErrLen)
{
	int result,BoardType;
	CString OutFile;
	CString BindTo;


	if (Thread<=0 || Thread>7) 
	{
		CStringW s;
		s.Format(Translate("Invalid Thread Number %d Valid Range (1-7)"),Thread);
		wcscpy_s(Err, MaxErrLen, s);
		return 1;
	}
	
	ConvertToOut(Thread,Name,OutFile.GetBuffer(MAX_PATH),MAX_PATH);
	OutFile.ReleaseBuffer();

	if (CheckKMotionVersion(&BoardType)) return 1;

	if (BoardType == BOARD_TYPE_UNKNOWN)  // Can't determine board type?
	{
		if (MaxErrLen > 0)
			wcscpy_s(Err, MaxErrLen, Translate("Unable to determine Controller Board Type"));

		return 1;
	}

	// Compile the C File

	result = Compile(Name,OutFile,BoardType,Thread,Err,MaxErrLen);
	if (result) return result;

	// Download the .out File

	result = LoadCoff(Thread, OutFile);
	if (result) return result;


	return 0;
}

int CKMotionDLL::Compile(const char* Name, const char* OutFile, const int BoardType, int Thread, char *Err, int MaxErrLen)
{
	wchar_t* ErrW = new wchar_t[MaxErrLen];
	*ErrW = 0;
	int rslt = Compile(Name, OutFile, BoardType, Thread, ErrW, MaxErrLen);
	strcpy_s(Err, MaxErrLen, CW2A(ErrW));
	delete ErrW;
	return rslt;
}

int CKMotionDLL::Compile(const char* Name, const char* OutFile, const int BoardType, int Thread, wchar_t* Err, int MaxErrLen)
{
	SECURITY_ATTRIBUTES sa          = {0};
	STARTUPINFO         si          = {0};
	PROCESS_INFORMATION pi          = {0};
	HANDLE              hPipeOutputRead  = NULL;
	HANDLE              hPipeOutputWrite = NULL;
	HANDLE              hPipeInputRead   = NULL;
	HANDLE              hPipeInputWrite  = NULL;
	BOOL                bTest = 0;
	DWORD               dwNumberOfBytesRead = 0;
	wchar_t             szMsg[100];

	CString Errors;

	if (Thread==0) return 1;

	// Check for #Pragma TI_COMPILER

	FILE *f=fopen(Name, "rt");

	if (f)
	{
		CString s;
		fgets(s.GetBufferSetLength(200), 200, f);
		s.ReleaseBuffer();
		fclose(f);

		RemoveComments(s);
		s.TrimLeft();
		int i = s.Find("#pragma");
		if (i >= 0)
		{
			int k = s.Find("TI_COMPILER",i);
			if (k > i)
			{
				return CompileTI(Name, OutFile, BoardType, Thread, Err, MaxErrLen);
			}
		}
	}


	// Try and locate the TCC67 Compiler
	CString Compiler = MainPathDLL + COMPILER;
	CString OFile=OutFile;

	f=fopen(Compiler,"r");  // try where the KMotionDLL was first

	if (f==NULL)
	{
		Compiler = MainPath + "\\Release" + COMPILER;
	
		f=fopen(Compiler,"r");  // try in the released directory next
		if (f==NULL)
		{
			Compiler = MainPath + "\\Debug" + COMPILER;
			f=fopen(Compiler,"r");  // try in the debug directory next
			if (f==NULL)
			{
				DoErrMsg(Translate("Error Locating TCC67.exe Compiler"));
				return 1;
			}
		}
	}
	fclose(f);

	
	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;

	// Create pipe for standard output redirection.
	CreatePipe(&hPipeOutputRead,  // read handle
		&hPipeOutputWrite, // write handle
		&sa,      // security attributes
		0      // number of bytes reserved for pipe - 0 default
		);
	
	// Create pipe for standard input redirection.
	CreatePipe(&hPipeInputRead,  // read handle
		&hPipeInputWrite, // write handle
		&sa,      // security attributes
		0      // number of bytes reserved for pipe - 0 default
		);
	
	// Make child process use hPipeOutputWrite as standard out,
	// and make sure it does not show on screen.
	si.cb = sizeof(si);
	si.dwFlags     = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
	si.wShowWindow = SW_HIDE;
	si.hStdInput   = hPipeInputRead;
	si.hStdOutput  = hPipeOutputWrite;
	si.hStdError   = hPipeOutputWrite;

	CString cmd;  // build command line
	CString BindTo,IncSrcPath1, IncSrcPath2, IncSrcPath3;

//ExcludeTranslate

	if (BoardType == BOARD_TYPE_KOGNA)
		IncSrcPath1 = "-I \"" + MainPathRoot + "\\DSP_KOGNA\" ";
	else
		IncSrcPath1="-I \"" + MainPathRoot + "\\DSP_KFLOP\" ";

	IncSrcPath2="-I \"" + ExtractPath(Name) + "\" ";

	if (BoardType == BOARD_TYPE_KOGNA)
		BindTo = MainPathRoot + "\\DSP_KOGNA\\DSPKOGNA.out";
	else 
		BindTo = MainPathRoot + "\\DSP_KFLOP\\DSPKFLOP.out";

	cmd.Format(" -text %08X -g -nostdinc " + IncSrcPath1 + IncSrcPath2 + IncSrcPath3 + "-o ",GetLoadAddress(Thread,BoardType));
	cmd = Compiler + cmd;
	cmd += "\"" + OFile + "\" \"" + Name + "\" \"" + BindTo +"\"";
	
	CreateProcess (
		NULL,
		cmd.GetBuffer(0), 
		NULL, NULL,
		TRUE, 0,
		NULL, NULL,
		&si, &pi);
//ResumeTranslate
	
	// Now that handles have been inherited, close it to be safe.
	// You don't want to read or write to them accidentally.
	CloseHandle(hPipeOutputWrite);
	CloseHandle(hPipeInputRead);
	
	
	// Wait for CONSPAWN to finish.
	WaitForSingleObject (pi.hProcess, INFINITE);

	DWORD exitcode;
	int result = GetExitCodeProcess(pi.hProcess,&exitcode);
	
	// Now test to capture DOS application output by reading
	// hPipeOutputRead.  Could also write to DOS application
	// standard input by writing to hPipeInputWrite.

	if (exitcode==0)
	{
		Errors="";
	}
	else
	{
		char *s = Errors.GetBuffer(10001);
		
		bTest=ReadFile(
			hPipeOutputRead,      // handle of the read end of our pipe
			s,					  // address of buffer that receives data
			10000,                // number of bytes to read
			&dwNumberOfBytesRead, // address of number of bytes read
			NULL                  // non-overlapped.
			);

		
		if (!bTest)
		{
			wsprintfW(szMsg, Translate("Error #%d reading compiler output."),GetLastError());
			DoErrMsg(szMsg);
			return 1;
		}

		// do something with data.
		s[dwNumberOfBytesRead] = 0;  // null terminate
		Errors.ReleaseBuffer();
	}


	
	// Close all remaining handles
	CloseHandle (pi.hProcess);
    CloseHandle (pi.hThread);
	CloseHandle (hPipeOutputRead);
	CloseHandle (hPipeInputWrite);
	
	/*----------Console application (CONSPAWN.EXE) code------*/

	CStringW ErrorsW = Errors;
	if (Err)
	{
		if (MaxErrLen > 0)
		{
			if (ErrorsW.GetLength() >= MaxErrLen-1) // too large to return?
				ErrorsW = ErrorsW.Left(MaxErrLen-1); // yes, trim excess
			wcscpy_s(Err, MaxErrLen, ErrorsW);
		}
	}

	return exitcode;
}

void CKMotionDLL::RemoveComments(CString &s)
{
	int i = s.Find("//");
	if (i >= 0)
		s = s.Left(i);

	i = s.Find("/*");
	if (i >= 0)
	{
		s = s.Left(s.GetLength() - i);

		int k = s.Find("*/", i);

		if (k > i)
		{   // both found remove section
			s.Delete(i, k - i + 2);
		}
		else // one found - remove remainder
		{
			s = s.Left(i);
		}
	}
}

int CKMotionDLL::CompileTI(const char * Name, const char * OutFile, const int BoardType, int Thread, wchar_t * Err, int MaxErrLen)
{
	SECURITY_ATTRIBUTES sa = { 0 };
	STARTUPINFO         si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	HANDLE              hPipeOutputRead = NULL;
	HANDLE              hPipeOutputWrite = NULL;
	HANDLE              hPipeInputRead = NULL;
	HANDLE              hPipeInputWrite = NULL;
	BOOL                bTest = 0;
	DWORD               dwNumberOfBytesRead = 0;
	wchar_t             szMsg[100];

	CStringW Errors;

	int Opt = 0; // default no optimization
	int MaxSize, BoardMaxSize;

	if (BoardType == BOARD_TYPE_KFLOP)
	{
		MaxSize = MAX_USER_PROG_SIZE_KFLOP;
		BoardMaxSize = 11 * 0x10000;
		if (Thread == 7) MaxSize *= 5;
	}
	else
	{
		MaxSize = MAX_USER_PROG_SIZE_KOGNA;
		BoardMaxSize = 10 * 0x40000;
		if (Thread == 7) MaxSize *= 4;

	}

	FILE *f = fopen(Name, "rt");

	if (f)
	{
		CString s;
		fgets(s.GetBufferSetLength(200), 200, f);
		s.ReleaseBuffer();
		fclose(f);

		RemoveComments(s);
		s.TrimLeft();
		int i = s.Find("#pragma");
		if (i >= 0)
		{
			int k = s.Find("TI_COMPILER", i);
			if (k > i)
			{
				CString sOpt, sMaxSize;
				int r1 = 1, r2 = 1, i1, i2, i3, i0;
				i0 = s.Find("(", k);  // look for (Opt) or (Opt,MaxSize)
				if (i0 > k)
				{
					i1 = s.Find(",", i0);  // look for (Opt) or (Opt,MaxSize)
					if (i1 > i0)
					{
						i2 = s.Find(")", i1);  //  (Opt,MaxSize)
						if (i2 > i1)
						{
							sOpt = s.Mid(i0 + 1, i1 - i0 - 1);
							r1 = sscanf(sOpt, "%d", &Opt);
							sMaxSize = s.Mid(i1 + 1, i2 - i1 - 1);

							i3 = sMaxSize.Find("0x");
							if (i3 >= 0)
							{	// hex value
								sMaxSize = sMaxSize.Right(sMaxSize.GetLength() - i3 - 2);
								r2 = sscanf(sMaxSize, "%x", &MaxSize);
							}
							else
							{	// decimal value  
								sMaxSize = s.Mid(i1 + 1, i2 - i1 - 1);
								r2 = sscanf(sMaxSize, "%d", &MaxSize);
							}
						}
					}
					else
					{
						r1 = 0;
						i1 = s.Find(")", i0);  // (Opt)
						if (i1 > i0)
						{
							sOpt = s.Mid(i0 + 1, i1 - i0 - 1);
							r1 = sscanf(sOpt, "%d", &Opt);
						}
					}
				}


				if (r1 != 1 || r2 != 1 || Opt < 0 || Opt > 3 || MaxSize <= 0 || MaxSize > BoardMaxSize)
				{
					// invalid format pragma TI_COMPILER

					Errors = Translate("Error Line :1: invalid TI_COMPILER pragma \n") + (CStringW) s +
						Translate("\n Expected format:\n#pragma TI_COMPILER\n#pragma TI_COMPILER(opt level 0-3)\n#pragma TI_COMPILER(opt level 0-3, Max Thread Space - hex or decimal)");

					if (MaxSize > BoardMaxSize)
						Errors += "\n\nInvalid Max Thread Space";

					CStringW ErrorsW = Errors;
					wcscpy_s(Err, MaxErrLen, ErrorsW);
					return 1;
				}
			}
		}
	}

	CString CompilerTI;
	if (BoardType == BOARD_TYPE_KFLOP)
	{
		CompilerTI = COMPILERTI_KFLOP;
	}
	else
	{
		CompilerTI = COMPILERTI_KOGNA;
	}


	// Try and locate the Compiler
	CString Compiler = MainPathRoot + CompilerTI;
	CString OFile = OutFile;

	f = fopen(Compiler, "r");  // try where the KMotionDLL was first

	if (f == NULL)
	{
		DoErrMsg(Translate("Error Locating cl6x.exe Compiler"));
		return 1;
	}

	fclose(f);

	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;

	// Create pipe for standard output redirection.
	CreatePipe(&hPipeOutputRead,  // read handle
		&hPipeOutputWrite, // write handle
		&sa,      // security attributes
		10000000      // number of bytes reserved for pipe - 0 default
	);

	// Create pipe for standard input redirection.
	CreatePipe(&hPipeInputRead,  // read handle
		&hPipeInputWrite, // write handle
		&sa,      // security attributes
		10000000      // number of bytes reserved for pipe - 0 default
	);

	// Make child process use hPipeOutputWrite as standard out,
	// and make sure it does not show on screen.
	si.cb = sizeof(si);
	si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
	si.wShowWindow = SW_HIDE;
	si.hStdInput = hPipeInputRead;
	si.hStdOutput = hPipeOutputWrite;
	si.hStdError = hPipeOutputWrite;

	CString cmd;  // build command line
	CString IncSrcPath1, IncSrcPath2, IncSrcPath3, opt, FileType;

	if (BoardType == BOARD_TYPE_KOGNA)
	{
		FileType = "--abi=eabi";
		IncSrcPath1 = "-i \"" + MainPathRoot + "\\DSP_KOGNA\" ";
	}
	else
	{
		IncSrcPath1 = "-i \"" + MainPathRoot + "\\DSP_KFLOP\" ";
	}

	IncSrcPath2 = "-i \"" + ExtractPath(Name) + "\" ";

	IncSrcPath3 = "-i \"" + MainPathRoot + "\\C Programs\" ";

// must use far data model as User programs call Firmware that uses its Near Data Pointer
	cmd.Format(" %s -k -q -as --diag_suppress=163 --mem_model:data=far "
		+ IncSrcPath1 + IncSrcPath2 + IncSrcPath3 + "-mu -ml3 -mv6710 -o%d", FileType, Opt);
	cmd = Compiler + cmd + " \"" + Name + "\" --obj_directory=\"" + ExtractPath(Name) + "\" --asm_directory=\"" + ExtractPath(Name) + "\"";

	CreateProcess(
		NULL,
		cmd.GetBuffer(0),
		NULL, NULL,
		TRUE, 0,
		NULL, NULL,
		&si, &pi);


	// Now that handles have been inherited, close it to be safe.
	// You don't want to read or write to them accidentally.
	CloseHandle(hPipeOutputWrite);
	CloseHandle(hPipeInputRead);


	// Wait for CONSPAWN to finish.
	WaitForSingleObject(pi.hProcess, INFINITE);

	DWORD exitcode;
	int result = GetExitCodeProcess(pi.hProcess, &exitcode);

	// Now test to capture DOS application output by reading
	// hPipeOutputRead.  Could also write to DOS application
	// standard input by writing to hPipeInputWrite.

	CString cs;
	char *s = cs.GetBuffer(10001);
	dwNumberOfBytesRead = 0;

	bTest = ReadFile(
		hPipeOutputRead,      // handle of the read end of our pipe
		s,					  // address of buffer that receives data
		10000,                // number of bytes to read
		&dwNumberOfBytesRead, // address of number of bytes read
		NULL                  // non-overlapped.
	);

	// do something with data.
	s[dwNumberOfBytesRead] = 0;  // null terminate
	cs.ReleaseBuffer();
	Errors = cs;

	// Close all remaining handles
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);
	CloseHandle(hPipeOutputRead);
	CloseHandle(hPipeInputWrite);

	if (!bTest && GetLastError() != ERROR_BROKEN_PIPE) // Note broken pipe just means there was nothing to read
	{
		wsprintfW(szMsg, Translate("Error #%d reading compiler output."), GetLastError());
		DoErrMsg(szMsg);
		return 1;
	}


	if (exitcode == 0)  // compile successful ?
	{
		CStringW LinkMessages;
		exitcode = LinkTI(Compiler, Name, OutFile, BoardType, Thread, LinkMessages.GetBufferSetLength(2000), 2000, MaxSize);
		LinkMessages.ReleaseBuffer();

		// Combine messages
		Errors = Errors + LinkMessages;
	}


	/*----------Console application (CONSPAWN.EXE) code------*/

	if (Err && MaxErrLen > 0)
	{
		if (Errors.GetLength() >= MaxErrLen - 1) // too large to return?
			Errors = Errors.Left(MaxErrLen - 1); // yes, trim excess
		wcscpy_s(Err, MaxErrLen, Errors);
	}


	return exitcode;
}

int CKMotionDLL::LinkTI(const char * Linker, const char * Name, const char * OutFile, const int BoardType, int Thread, wchar_t * Err, int MaxErrLen, int MaxSize)
{
	SECURITY_ATTRIBUTES sa = { 0 };
	STARTUPINFO         si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	HANDLE              hPipeOutputRead = NULL;
	HANDLE              hPipeOutputWrite = NULL;
	HANDLE              hPipeInputRead = NULL;
	HANDLE              hPipeInputWrite = NULL;
	BOOL                bTest = 0;
	DWORD               dwNumberOfBytesRead = 0;
	wchar_t                szMsg[100];

	CStringW Errors;

	CString OFile = OutFile;

	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;

	// Create pipe for standard output redirection.
	CreatePipe(&hPipeOutputRead,  // read handle
		&hPipeOutputWrite, // write handle
		&sa,      // security attributes
		10000000      // number of bytes reserved for pipe - 0 default
	);

	// Create pipe for standard input redirection.
	CreatePipe(&hPipeInputRead,  // read handle
		&hPipeInputWrite, // write handle
		&sa,      // security attributes
		10000000      // number of bytes reserved for pipe - 0 default
	);

	// Make child process use hPipeOutputWrite as standard out,
	// and make sure it does not show on screen.
	si.cb = sizeof(si);
	si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
	si.wShowWindow = SW_HIDE;
	si.hStdInput = hPipeInputRead;
	si.hStdOutput = hPipeOutputWrite;
	si.hStdError = hPipeOutputWrite;

	CString CompilerTI, LinkTemplate, Symbols, LinkCmd;
	if (BoardType == BOARD_TYPE_KFLOP)
	{
		CompilerTI = COMPILERTI_KFLOP;
		LinkTemplate = LINKTEMPLATE_KFLOP;
		Symbols = SYMBOLS_KFLOP;
		LinkCmd = TILINKCMD_KFLOP;
	}
	else
	{
		CompilerTI = COMPILERTI_KOGNA;
		LinkTemplate = LINKTEMPLATE_KOGNA;
		Symbols = SYMBOLS_KOGNA;
		LinkCmd = TILINKCMD_KOGNA;
	}

	CString cmd;  // build command line
	cmd = (CString)Linker + " -z " + "\"" + MainPathRoot + LinkCmd + "\"";

	CString LinkTemplateData;
	
	FILE *f = fopen(MainPathRoot + LinkTemplate, "rt");
	
	if (!f)
	{
		DoErrMsg(Translate("Unable to open TI Link Template.\n\n") + (CStringW)MainPathRoot + (CStringW)LinkTemplate);
		return 1;
	}

	int n = (int)fread_s(LinkTemplateData.GetBufferSetLength(100000), 100000, 1, 100000, f);
	LinkTemplateData.ReleaseBufferSetLength(n);
	fclose(f);


	CString MapFile, ObjFile = Name;
	n = ObjFile.ReverseFind('.');
	if (n > 0) ObjFile.Delete(n, ObjFile.GetLength() - n);
	MapFile = ObjFile + ".map";
	ObjFile = ObjFile + ".obj";


	// Find IRAM_END in symbols file
	f = fopen(MainPathRoot + Symbols, "rt");
	if (!f)
	{
		DoErrMsg(Translate("Error unable to open DSP Symbol file.\n\n") + (CStringW)MainPathRoot + (CStringW)Symbols);
		return 1;
	}
	CString SymbolsData;
	n = (int)fread_s(SymbolsData.GetBufferSetLength(100000), 100000, 1, 100000, f);
	SymbolsData.ReleaseBufferSetLength(n);
	fclose(f);

	int i0 = SymbolsData.Find("IRAM_END=0x");
	if (i0 == -1)
	{
		DoErrMsg(Translate("Error unable to find IRAM_END sysmbol to determine IRAM size"));
		return 1;
	}

	int i1 = SymbolsData.Find(';',i0+11);
	if (i1 == -1)
	{
		DoErrMsg(Translate("Error unable to find IRAM_END sysmbol to determine IRAM size"));
		return 1;
	}

	CString IramEnd = SymbolsData.Mid(i0+11, i1 - i0);
	int iRamEnd;
	int r = sscanf(IramEnd, "%x", &iRamEnd);
	if (r != 1)
	{
		DoErrMsg(Translate("Error unable to find IRAM_END sysmbol to determine IRAM size"));
		return 1;
	}

//ExcludeTranslate
	CString IR,IRL,TS,TL;
	IR.Format("0x%08x", iRamEnd);
	if (BoardType == BOARD_TYPE_KFLOP)
		IRL.Format("0x%08x", 0x10020000-iRamEnd);
	else
		IRL.Format("0x%08x", 0x11840000 - iRamEnd);
	TS.Format("0x%08x", GetLoadAddress(Thread, BoardType));
	TL.Format("0x%08x", MaxSize);

	LinkTemplateData.Replace("{OBJECTFILE}", "\"" + ObjFile + "\"");
	LinkTemplateData.Replace("{MAPFILE}", "\"" + MapFile + "\"");
	LinkTemplateData.Replace("{OUTPUTFILE}", "\"" + (CString)OutFile + "\"");
	LinkTemplateData.Replace("{IRAMSTART}", IR);
	LinkTemplateData.Replace("{IRAMLENGTH}", IRL);
	LinkTemplateData.Replace("{THREADSTART}", TS);
	LinkTemplateData.Replace("{THREADLENGTH}", TL);
	if (BoardType == BOARD_TYPE_KFLOP)
		LinkTemplateData.Replace("{DSP_KFLOP_PATH}", MainPathRoot + "\\DSP_KFLOP\\");
	else
		LinkTemplateData.Replace("{DSP_KFLOP_PATH}", MainPathRoot + "\\DSP_KOGNA\\");
//ResumeTranslate

	f = fopen(MainPathRoot + LinkCmd, "wt");

	if (!f)
	{
		DoErrMsg(Translate("Unable to open TI Link Command File") + (CStringW)MainPathRoot + (CStringW)LinkCmd);
		return 1;
	}

	n = (int)fwrite(LinkTemplateData,LinkTemplateData.GetLength(), 1, f);
	fclose(f);

	CreateProcess(
		NULL,
		cmd.GetBuffer(0),
		NULL, NULL,
		TRUE, 0,
		NULL, NULL,
		&si, &pi);


	// Now that handles have been inherited, close it to be safe.
	// You don't want to read or write to them accidentally.
	CloseHandle(hPipeOutputWrite);
	CloseHandle(hPipeInputRead);


	// Wait for CONSPAWN to finish.
	WaitForSingleObject(pi.hProcess, INFINITE);

	DWORD exitcode;
	int result = GetExitCodeProcess(pi.hProcess, &exitcode);

	// Now test to capture DOS application output by reading
	// hPipeOutputRead.  Could also write to DOS application
	// standard input by writing to hPipeInputWrite.

	CString cs;
	char *s = cs.GetBuffer(10001);

	bTest = ReadFile(
		hPipeOutputRead,      // handle of the read end of our pipe
		s,					  // address of buffer that receives data
		10000,                // number of bytes to read
		&dwNumberOfBytesRead, // address of number of bytes read
		NULL                  // non-overlapped.
	);


	if (!bTest)
	{
		wsprintfW(szMsg, Translate("Error #%d reading compiler output."), GetLastError());
		DoErrMsg(szMsg);
		return 1;
	}

	// do something with data.
	s[dwNumberOfBytesRead] = 0;  // null terminate
	cs.ReleaseBuffer();
	Errors = cs;


	// Close all remaining handles
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);
	CloseHandle(hPipeOutputRead);
	CloseHandle(hPipeInputWrite);

	/*----------Console application (CONSPAWN.EXE) code------*/

	if (Err && MaxErrLen > 0)
	{
		if (Errors.GetLength() >= MaxErrLen - 1) // too large to return?
			Errors = Errors.Left(MaxErrLen - 1); // yes, trim excess
		wcscpy_s(Err, MaxErrLen, Errors);
	}

	return exitcode;
}

int CKMotionDLL::ValidateC(const char *Name, wchar_t *Err, int MaxErrLen)
{
	SECURITY_ATTRIBUTES sa = { 0 };
	STARTUPINFO         si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	HANDLE              hPipeOutputRead = NULL;
	HANDLE              hPipeOutputWrite = NULL;
	HANDLE              hPipeInputRead = NULL;
	HANDLE              hPipeInputWrite = NULL;
	BOOL                bTest = 0;
	DWORD               dwNumberOfBytesRead = 0;
	wchar_t             szMsg[100];



	// Try and locate splint

	CStringW Errors;
	CString Compiler = MainPathRoot + VALIDATOR;

	FILE *f = fopen(Compiler, "r");  // try where the KMotionDLL was irst

	if (f == NULL)
	{
		DoErrMsg(Translate("Error Locating Cppcheck.exe code validator"));
		return 1;
	}
	fclose(f);


	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;




	// Create pipe for standard output redirection.
	CreatePipe(&hPipeOutputRead,  // read handle
		&hPipeOutputWrite, // write handle
		&sa,      // security attributes
		1000000   // number of bytes reserved for pipe - 0 default
	);

	// Create pipe for standard input redirection.
	CreatePipe(&hPipeInputRead,  // read handle
		&hPipeInputWrite, // write handle
		&sa,      // security attributes
		1000000   // number of bytes reserved for pipe - 0 default
	);

	// Make child process use hPipeOutputWrite as standard out,
	// and make sure it does not show on screen.
	si.cb = sizeof(si);
	si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
	si.wShowWindow = SW_HIDE;
	si.hStdInput = hPipeInputRead;
	si.hStdOutput = hPipeOutputWrite;
	si.hStdError = hPipeOutputWrite;
//ExcludeTranslate

	CString cmd;  // build command line
	CString BindTo, IncSrcPath1, IncSrcPath2, IncSrcPath3, Susppress, opts, templ;

	IncSrcPath1 = "-I \"" + MainPathRoot + "\\DSP_KFLOP\" ";
	IncSrcPath2 = "-I \"" + ExtractPath(Name) + "\" ";
	IncSrcPath3 = "-I \"" + MainPathRoot + "\\C Programs\" ";
	Susppress = "--suppressions-list=\"" + MainPathRoot + VALIDATOR_SUPPRESS + "\" ";
	opts = "--enable=all ";
	templ = "--template=\"{file}, line {line} : {severity} {id}\n{message}\n{code}\n\" ";
	
	cmd = Compiler + " \"" + Name + "\" " + opts + IncSrcPath1 + IncSrcPath2 + IncSrcPath3 + Susppress + templ;

//ResumeTranslate

	CreateProcess(
		NULL,
		cmd.GetBuffer(0),
		NULL, NULL,
		TRUE, 0,
		NULL, NULL,
		&si, &pi);


	// Now that handles have been inherited, close it to be safe.
	// You don't want to read or write to them accidentally.
	CloseHandle(hPipeOutputWrite);
	CloseHandle(hPipeInputRead);


	bool Done = true;
	DWORD Timeout;

	do
	{
		// Wait for CONSPAWN to finish.
		Timeout = WaitForSingleObject(pi.hProcess, 3000);  // timeout in 30 seconds

		Done = Timeout == WAIT_OBJECT_0 || MessageBoxW(NULL, Translate("splint taking a long time for validation.  Continue waiting?"), L"KMotion", MB_YESNO) == IDNO;
	} while (!Done);

	DWORD exitcode;
	int result = GetExitCodeProcess(pi.hProcess, &exitcode);

	if (Timeout != WAIT_OBJECT_0)
	{
		exitcode = 0; // treat stop waiting as success
		TerminateProcess(pi.hProcess, 9999);
	}

	// Now test to capture DOS application output by reading
	// hPipeOutputRead.  Could also write to DOS application
	// standard input by writing to hPipeInputWrite.

	CString cs;
	char *s = cs.GetBuffer(10001);

	bTest = ReadFile(
		hPipeOutputRead,      // handle of the read end of our pipe
		s,					  // address of buffer that receives data
		10000,                // number of bytes to read
		&dwNumberOfBytesRead, // address of number of bytes read
		NULL                  // non-overlapped.
	);


	if (!bTest)
	{
		wsprintfW(szMsg, Translate("Error #%d reading Validation output."), GetLastError());
		DoErrMsg(szMsg);
		return 1;
	}

	// do something with data.
	s[dwNumberOfBytesRead] = 0;  // null terminate
	cs.ReleaseBuffer();
	Errors = cs;

	if (exitcode != 0)
	{
		Errors = Translate("Validation Failed!\n") + Errors;
	}


	if (Timeout != WAIT_OBJECT_0) Errors = Errors + Translate(" Validation timed out");



	// Close all remaining handles
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);
	CloseHandle(hPipeOutputRead);
	CloseHandle(hPipeInputWrite);

	/*----------Console application (CONSPAWN.EXE) code------*/

	if (Err && MaxErrLen > 0)
	{
		if (Errors.GetLength() >= MaxErrLen - 1) // too large to return?
			Errors = Errors.Left(MaxErrLen - 1); // yes, trim excess
		wcscpy_s(Err, MaxErrLen, Errors);
	}

	return exitcode;
}


void CKMotionDLL::ConvertToOut(int thread, const char *InFile, char *OutFile, int MaxLength)
{
	CString OFile;
	CString IFile=InFile;

	CString InFileWithCase=InFile;
	CString ThreadString;

	if (thread>0)
	{

		ThreadString.Format("(%d).out",thread);

		IFile.MakeLower();

		if (IFile.Right(2)= ".c")
		{
			OFile = InFileWithCase.Left(InFileWithCase.GetLength()-2);
		}
		else if (IFile.Find(".txt")!=-1)
		{
			OFile = InFileWithCase.Left(InFileWithCase.GetLength()-4);
		}
		else if (IFile.Find(".cpp")!=-1)
		{
			OFile = InFileWithCase.Left(InFileWithCase.GetLength()-4);
		}
		else
		{
			OFile = InFileWithCase;
		}

		OFile += ThreadString;
	}

	strncpy(OutFile,OFile,MaxLength);
}


CString CKMotionDLL::ExtractPath(CString InFile)
{
	int next_pos=0,pos;

	CString OutFile;

	do
	{
		pos=next_pos;
		next_pos = InFile.Find('\\',pos+1);
	}
	while (next_pos!=-1);

	if (pos>0)
		OutFile = InFile.Left(pos);
	else
		OutFile = "";

	return OutFile;
}

unsigned int CKMotionDLL::GetLoadAddress(int thread, int BoardType) 
{
	if (BoardType == BOARD_TYPE_KOGNA)
		return USER_PROG_ADDRESS_KOGNA + (thread - 1) * MAX_USER_PROG_SIZE_KOGNA;
	else if (BoardType == BOARD_TYPE_KFLOP)
		return USER_PROG_ADDRESS_KFLOP + (thread-1) * MAX_USER_PROG_SIZE_KFLOP;
	else
	{
		DoErrMsg(Translate("Error Board Type"));
		return (0xfffffff);
	}
}

// To avoid GUI deadlocks if called from GUI and not essential to determine
// the Board Type set Wait (default) to work on best effort manner


int CKMotionDLL::CheckKMotionVersion(int *type, bool GetBoardTypeOnly, bool Wait) 
{
	int result;
	CString BoardVersion;
	CString CoffVersion;
	CString OutFile;
	CStringW ms;

	if (type) *type = BOARD_TYPE_UNKNOWN;

	if (Wait)
		result = WaitToken("CheckKMotionVersion");
	else
		result = KMotionLock("CheckKMotionVersion");

	if (result == KMOTION_LOCKED)  // see if we can get access
	{
		// Get the firmware date from the KMotion Card which
		// will be in PT (Pacific Time)
		ReleaseToken();
		result = WriteLineReadLine("Version",BoardVersion.GetBufferSetLength(MAX_LINE));
		BoardVersion.ReleaseBuffer();
		if (result) return result;

		// now get date stamp from firmware .out file

		if (BoardVersion.Find("KOGNA") == 0)
		{
			OutFile = MainPathRoot + "\\DSP_KOGNA\\DSPKOGNA.out";
			if (type) *type = BOARD_TYPE_KOGNA;
		}
		else
		{
			OutFile = MainPathRoot +"\\DSP_KFLOP\\DSPKFLOP.out";
			if (type) *type = BOARD_TYPE_KFLOP;
		}

		if (GetBoardTypeOnly) return 0;

		int result = ExtractCoffVersionString(OutFile,CoffVersion.GetBuffer(81));	
		CoffVersion.ReleaseBuffer();

		if (result)
		{
			ms.Format(Translate("Error Extracting Version Information from file\r\r") + L" %s",	(CStringW)OutFile);
			DoErrMsg(ms);
			return 1;
		}

		CoffVersion.Remove('\n');
		

		// check if they match exactly

		if (CoffVersion == BoardVersion) return 0;

		if (BoardVersion.Find("KOGNA") == 0)
			ms.Format(Translate("DSP_KOGNA.out Date Stamp Doesn't match KOGNA Firmware\r\r Before compiling programs please use Flash/Config Screen and select:\r Download New Version.  This will install compatible Firmware with\r this version of software\r\r %s  KOGNA Firmware\r %s  DSP_KOGNA.out file"),
				(CStringW)BoardVersion,
				(CStringW)CoffVersion);
		else if (BoardVersion.Find("KFLOP") == 0)
			ms.Format(Translate("DSP_KFLOP.out Date Stamp Doesn't match KFLOP Firmware\r\r Before compiling programs please use Flash/Config Screen and select:\r Download New Version.  This will install compatible Firmware with\r this version of software\r\r %s  KFLOP Firmware\r %s  DSP_KFLOP.out file"),
				(CStringW)BoardVersion,
				(CStringW)CoffVersion);
		else
			ms.Format(Translate("DSP_KMotion.out Date Stamp Doesn't match KMotion Firmware\r\r Before compiling programs please use Flash/Config Screen and select:\r Download New Version.  This will install compatible Firmware with\r this version of software\r\r %s  KMotion Firmware\r %s  DSP_KMotion.out file"),
				(CStringW)BoardVersion,
				(CStringW)CoffVersion);


		
		DoErrMsg(ms);

		return 1;
	}

	return 0;
}



// return size of coff file sections

int CKMotionDLL::CheckCoffSize(const char *InFile, int *size_text, int *size_bss, int *size_data, int *size_total)
{
	FILE *f;
	unsigned int str_size;
	char *Coff_str_table, *name;
	int i,k;
	syment csym;
	char name2[9];
	unsigned int start_text=0, end_text=0, start_bss=0, end_bss=0, start_data=0, end_data=0; 
	unsigned int min,max;

	
	f = fopen(InFile,"rb");

	if (!f) return 1;

	if (fread(&file_hdr, FILHSZ, 1, f) != 1) return 1;

/*-------------------------------------------------------------------------*/
/* MAKE SURE THIS IS REALLY A COFF FILE. 
/*-------------------------------------------------------------------------*/
	if (file_hdr.f_magic != MAGIC)
	{
		fclose(f);
		// its, not try as Elf
		return CheckElfSize(InFile, size_text, size_bss, size_data, size_total);
	}

	if (fread(&o_filehdr, sizeof(o_filehdr), 1, f) != 1) return 1;

	// first read the string table

	if (fseek(f,file_hdr.f_symptr + file_hdr.f_nsyms * SYMESZ,SEEK_SET)) return 1;
	if (fread(&str_size, sizeof(int), 1, f) != 1) return 1;

	Coff_str_table = (char *)malloc(str_size);

	if (fread(Coff_str_table, str_size-4, 1, f) != 1) {free(Coff_str_table); return 1;};

	// read/process all the symbols

	// seek back to symbols

	if (fseek(f,file_hdr.f_symptr,SEEK_SET)) {free(Coff_str_table); return 1;};

	for (i=0; i< file_hdr.f_nsyms; i++)
	{
		if (fread(&csym, SYMESZ, 1, f) != 1) {free(Coff_str_table); return 1;};

		if (csym._n._n_n._n_zeroes == 0)
		{
			name = Coff_str_table + csym._n._n_n._n_offset - 4 ;
		}
		else
		{
			name = csym._n._n_name;

			if (name[7] != 0)
			{
				for (k=0; k<8; k++)
					name2[k] = name[k];

				name2[8]=0;

				name = name2;
			}
		}

		// check for the names we are looking for

		if (strcmp("__start_.text",name)==0)  start_text = csym.n_value;
		if (strcmp("__stop_.text" ,name)==0)  end_text   = csym.n_value;
		if (strcmp("__start_.bss" ,name)==0)  start_bss  = csym.n_value;
		if (strcmp("__stop_.bss"  ,name)==0)  end_bss    = csym.n_value;
		if (strcmp("__start_.data",name)==0)  start_data = csym.n_value;
		if (strcmp("__stop_.data" ,name)==0)  end_data   = csym.n_value;

		// skip any aux records

		if (csym.n_numaux == 1)
		{
			if (fread(&csym, SYMESZ, 1, f) != 1) {free(Coff_str_table); return 1;};
			i++;
		}
	}

	fclose(f);
	free(Coff_str_table); 

	*size_text = end_text-start_text;
	*size_bss  = end_bss -start_bss;
	*size_data = end_data-start_data;
	
	
	min = 0xffffffff;
	if (start_text != 0 && min > start_text) min = start_text;
	if (start_bss  != 0 && min > start_bss ) min = start_bss;
	if (start_data != 0 && min > start_data) min = start_data;
	
	max = 0x0;
	if (end_text != 0 && max < end_text) max = end_text;
	if (end_bss  != 0 && max < end_bss ) max = end_bss;
	if (end_data != 0 && max < end_data) max = end_data;

	*size_total = max-min;
	
	return 0;
}

#define MAX_SECTIONS 200
#define SHT_PROGBITS 1
#define SHF_ALLOC 2

int CKMotionDLL::CheckElfSize(const char* InFile, int* size_text, int* size_bss, int* size_data, int* size_total)
{
	FILE* f;
	int i;
	unsigned int VersionAddress = 0;
	Elf32_Ehdr file_hdr;                   /* FILE HEADER STRUCTURE              */
	Elf32_Shdr sect_hdr, const_hdr, symbol_hdr, strtab_hdr;
	unsigned int start_text = 0, end_text = 0, start_bss = 0, end_bss = 0, start_data = 0, end_data = 0;
	unsigned int min, max;

	f = fopen(InFile, "rb");

	if (!f) return 1;

	if (fread(&file_hdr, sizeof(file_hdr), 1, f) != 1) return 1;

	// check for ELF file
	char* p = (char*)&file_hdr;
	if (p[1] != 'E' || p[2] != 'L' || p[3] != 'F')
	{
		fclose(f);
		return 1;  // not an elf file
	}

	// find the offset to the string table
	// read in the String section header
	if (fseek(f, file_hdr.e_shoff + file_hdr.e_shstrndx * file_hdr.e_shentsize, SEEK_SET)) return 1;
	if (fread(&sect_hdr, sizeof(sect_hdr), 1, f) != 1) return 1;

	int HeaderStringsOffset = sect_hdr.sh_offset;
	int HeaderStringTableSize = sect_hdr.sh_size;

	char* HeaderStringTable = (char*)malloc(HeaderStringTableSize + 1);
	if (fseek(f, HeaderStringsOffset, SEEK_SET)) { delete HeaderStringTable;  return 1; }
	if (fread(HeaderStringTable, HeaderStringTableSize, 1, f) != 1) { delete HeaderStringTable;  return 1; }


	//find 3 section headers
	const_hdr.sh_name = strtab_hdr.sh_name = symbol_hdr.sh_name = 0;

	if (fseek(f, file_hdr.e_shoff, SEEK_SET)) return 1;  // seek to section headers
	for (i = 0; i < file_hdr.e_shnum; i++)
	{
		if (fread(&sect_hdr, sizeof(sect_hdr), 1, f) != 1) { delete HeaderStringTable;  return 1; }

		if (strcmp(HeaderStringTable + sect_hdr.sh_name, ".const") == 0)
			const_hdr = sect_hdr;
		else if (strcmp(HeaderStringTable + sect_hdr.sh_name, ".strtab") == 0)
			strtab_hdr = sect_hdr;
		else if (strcmp(HeaderStringTable + sect_hdr.sh_name, ".symtab") == 0)
			symbol_hdr = sect_hdr;

		if (const_hdr.sh_name != 0 && strtab_hdr.sh_name != 0 && symbol_hdr.sh_name != 0) break;
	}

	delete HeaderStringTable;

	if (i == file_hdr.e_shnum) return 1;

	// Read in main String table
	int StringsOffset = strtab_hdr.sh_offset;
	int StringTableSize = strtab_hdr.sh_size;

	char* StringTable = (char*)malloc(StringTableSize + 1);
	if (fseek(f, StringsOffset, SEEK_SET)) { delete StringTable;  return 1; }
	if (fread(StringTable, StringTableSize, 1, f) != 1) { delete StringTable;  return 1; }

	// read in all symbols
	Elf32_Sym* Syms = (Elf32_Sym*)malloc(symbol_hdr.sh_size * sizeof(Elf32_Sym));
	if (fseek(f, symbol_hdr.sh_offset, SEEK_SET)) { delete StringTable; delete Syms;  return 1; }  // seek to symbols
	if (fread(Syms, symbol_hdr.sh_size, 1, f) != 1) { delete StringTable; delete Syms;  return 1; }

	// go through the symbols looking for the Version 
	int count = 0;
	for (i = 0; i < (int)(symbol_hdr.sh_size / sizeof(Elf32_Sym)); i++)
	{
		char* name = StringTable + Syms[i].st_name;
		if (strcmp("__start_.text", name) == 0)  { start_text = Syms[i].st_value; count++; }
		if (strcmp("__stop_.text", name) == 0)  { end_text = Syms[i].st_value; count++; }
		if (strcmp("__start_.bss", name) == 0)  { start_bss = Syms[i].st_value; count++; }
		if (strcmp("__stop_.bss", name) == 0)  { end_bss = Syms[i].st_value; count++; }
		if (strcmp("__start_.data", name) == 0)  { start_data = Syms[i].st_value; count++; }
		if (strcmp("__stop_.data", name) == 0)  { end_data = Syms[i].st_value; count++; }
	}

	delete StringTable;
	delete Syms;
	fclose(f);

	*size_text = end_text - start_text;
	*size_bss = end_bss - start_bss;
	*size_data = end_data - start_data;


	min = 0xffffffff;
	if (start_text != 0 && min > start_text) min = start_text;
	if (start_bss != 0 && min > start_bss) min = start_bss;
	if (start_data != 0 && min > start_data) min = start_data;

	max = 0x0;
	if (end_text != 0 && max < end_text) max = end_text;
	if (end_bss != 0 && max < end_bss) max = end_bss;
	if (end_data != 0 && max < end_data) max = end_data;

	*size_total = max - min;
	return 0;
}



// Find special version string that is pointed to by symbol
//
// VersionAndBuildTime
//
// within the  .const section of the COFF file
//


int CKMotionDLL::ExtractElfVersionString(const char *InFile, char *Version)
{
	FILE *f;
	int i;
	unsigned int VersionAddress = 0;
	Elf32_Ehdr file_hdr;                   /* FILE HEADER STRUCTURE              */
	Elf32_Shdr sect_hdr, const_hdr, symbol_hdr, strtab_hdr;


	f = fopen(InFile, "rb");


	if (!f) return 1;

	if (fread(&file_hdr, sizeof(file_hdr), 1, f) != 1) return 1;

	// find the offset to the string table
	// read in the String section header
	if (fseek(f, file_hdr.e_shoff + file_hdr.e_shstrndx * file_hdr.e_shentsize, SEEK_SET)) return 1;
	if (fread(&sect_hdr, sizeof(sect_hdr), 1, f) != 1) return 1;

	int HeaderStringsOffset = sect_hdr.sh_offset;
	int HeaderStringTableSize = sect_hdr.sh_size;

	char *HeaderStringTable = (char *)malloc(HeaderStringTableSize + 1);
	if (fseek(f, HeaderStringsOffset, SEEK_SET)) { delete HeaderStringTable;  return 1; }
	if (fread(HeaderStringTable, HeaderStringTableSize, 1, f) != 1) { delete HeaderStringTable;  return 1; }

	//find 3 section headers
	const_hdr.sh_name = strtab_hdr.sh_name = symbol_hdr.sh_name = 0;

	if (fseek(f, file_hdr.e_shoff, SEEK_SET)) return 1;  // seek to section headers
	for (i = 0; i < file_hdr.e_shnum; i++)
	{
		if (fread(&sect_hdr, sizeof(sect_hdr), 1, f) != 1) { delete HeaderStringTable;  return 1; }

		if (strcmp(HeaderStringTable + sect_hdr.sh_name, ".const") == 0)
			const_hdr = sect_hdr;
		else if (strcmp(HeaderStringTable + sect_hdr.sh_name, ".strtab") == 0)
			strtab_hdr = sect_hdr;
		else if (strcmp(HeaderStringTable + sect_hdr.sh_name, ".symtab") == 0)
			symbol_hdr = sect_hdr;

		if (const_hdr.sh_name != 0 && strtab_hdr.sh_name != 0 && symbol_hdr.sh_name != 0) break;
	}

	delete HeaderStringTable;

	if (i == file_hdr.e_shnum) return 1;

	// Read in main String table
	int StringsOffset = strtab_hdr.sh_offset;
	int StringTableSize = strtab_hdr.sh_size;

	char *StringTable = (char *)malloc(StringTableSize + 1);
	if (fseek(f, StringsOffset, SEEK_SET)) { delete StringTable;  return 1; }
	if (fread(StringTable, StringTableSize, 1, f) != 1) { delete StringTable;  return 1; }

	// read in all symbols
	Elf32_Sym *Syms = (Elf32_Sym *)malloc(symbol_hdr.sh_size * sizeof(Elf32_Sym));
	if (fseek(f, symbol_hdr.sh_offset, SEEK_SET)) { delete StringTable; delete Syms;  return 1; }  // seek to symbols
	if (fread(Syms, symbol_hdr.sh_size, 1, f) != 1) { delete StringTable; delete Syms;  return 1; }

	// go through the symbols looking for the Version 
	for (i = 0; i < (int)(symbol_hdr.sh_size / sizeof(Elf32_Sym)); i++)
	{
		if (strcmp(StringTable + Syms[i].st_name, "VersionAndBuildTime") == 0)
		{
			// found it
			// seek to it in .const
			if (fseek(f, Syms[i].st_value - const_hdr.sh_addr + const_hdr.sh_offset, SEEK_SET)) { delete StringTable; delete Syms;  return 1; }
			// read it
			if (fread(Version, 50, 1, f) != 1) { delete StringTable; delete Syms;  return 1; }
			delete StringTable;
			delete Syms;
			return 0;
		}
	}

	delete StringTable;
	delete Syms;
	return 0;
}


int CKMotionDLL::ExtractCoffVersionString(const char *InFile, char *Version)
{
	FILE *f;
	unsigned int str_size;
	char *Coff_str_table, *name;
	int i,k;
	syment csym;
	char name2[9];
	unsigned int VersionAddress=0; 
	SCNHDR sect_hdr;
	FILHDR  file_hdr;                       /* FILE HEADER STRUCTURE              */
	AOUTHDR o_filehdr;                      /* OPTIONAL (A.OUT) FILE HEADER       */

	f = fopen(InFile,"rb");

	if (!f) return 1;

	if (fread(&file_hdr, FILHSZ, 1, f) != 1) return 1;

	if (fread(&o_filehdr, sizeof(o_filehdr), 1, f) != 1) return 1;

	// check for ELF file
	char *p = (char*)&file_hdr;
	if (p[1] == 'E' && p[2] == 'L' &&p[3] == 'F')
	{
		fclose(f);
		return ExtractElfVersionString(InFile, Version);
	}

	// search for the .const section header

	for (i=0; i<file_hdr.f_nscns; i++)
	{
		if (fread(&sect_hdr, SCNHSZ, 1, f) != 1) return 1;

		if (strcmp(".const",sect_hdr.s_name)==0) break;  // found it?
	}

	if (i==file_hdr.f_nscns) return 1;


	// now read the string table

	if (fseek(f,file_hdr.f_symptr + file_hdr.f_nsyms * SYMESZ,SEEK_SET)) return 1;
	if (fread(&str_size, sizeof(int), 1, f) != 1) return 1;

	Coff_str_table = (char *)malloc(str_size);

	if (fread(Coff_str_table, str_size-4, 1, f) != 1) {free(Coff_str_table); return 1;};

	// read/process all the symbols

	// seek back to symbols

	if (fseek(f,file_hdr.f_symptr,SEEK_SET)) {free(Coff_str_table); return 1;};

	for (i=0; i< file_hdr.f_nsyms; i++)
	{
		if (fread(&csym, SYMESZ, 1, f) != 1) {free(Coff_str_table); return 1;};

		if (csym._n._n_n._n_zeroes == 0)
		{
			name = Coff_str_table + csym._n._n_n._n_offset - 4 ;
		}
		else
		{
			name = csym._n._n_name;

			if (name[7] != 0)
			{
				for (k=0; k<8; k++)
					name2[k] = name[k];

				name2[8]=0;

				name = name2;
			}
		}

		// check for the names we are looking for

		if (strcmp("_VersionAndBuildTime",name)==0)
		{
			VersionAddress = csym.n_value;
			break;
		}

		// skip any aux records

		if (csym.n_numaux == 1)
		{
			if (fread(&csym, SYMESZ, 1, f) != 1) {free(Coff_str_table); return 1;};
			i++;
		}
	}


	free(Coff_str_table); 

	if (VersionAddress==0) return 1;

	// compute file offset for the string

	int offset = sect_hdr.s_scnptr +         // file pointer to data
		(VersionAddress - sect_hdr.s_paddr); // plus offset - physical address


	if (fseek(f,offset,SEEK_SET)) return 1;
	
	for (i=0; i<80; i++)
	{
		if (fread(Version+i, sizeof(char), 1, f) != 1) return 1;
		if (Version[i]==0) break;
	}

	fclose(f);

	if (i==80) return 1;
	
	return 0;
}


int CKMotionDLL::ElfLoad(const char *InFile, unsigned int *EntryPoint, int PackToFlash)
{
	FILE *f;
	int i;
	unsigned int VersionAddress = 0;
	Elf32_Ehdr file_hdr;                   /* FILE HEADER STRUCTURE              */
	Elf32_Shdr sect_hdr, LoadSections[MAX_SECTIONS];
	int iLoadSections = 0;

	f = fopen(InFile, "rb");

	if (!f) return 1;

	if (fread(&file_hdr, sizeof(file_hdr), 1, f) != 1) return 1;

	// check for ELF file
	char *p = (char*)&file_hdr;
	if (p[1] != 'E' || p[2] != 'L' || p[3] != 'F')
	{
		fclose(f);
		return 1;  // not an elf file
	}

	// find the offset to the string table
	// read in the String section header
	if (fseek(f, file_hdr.e_shoff + file_hdr.e_shstrndx * file_hdr.e_shentsize, SEEK_SET)) return 1;
	if (fread(&sect_hdr, sizeof(sect_hdr), 1, f) != 1) return 1;

	int HeaderStringsOffset = sect_hdr.sh_offset;
	int HeaderStringTableSize = sect_hdr.sh_size;

	char *HeaderStringTable = (char *)malloc(HeaderStringTableSize + 1);
	if (fseek(f, HeaderStringsOffset, SEEK_SET)) { delete HeaderStringTable;  return 1; }
	if (fread(HeaderStringTable, HeaderStringTableSize, 1, f) != 1) { delete HeaderStringTable;  return 1; }


	*EntryPoint = file_hdr.e_entry; // return the entry point

	//identify all the sections that have data to be placed in memory

	if (fseek(f, file_hdr.e_shoff, SEEK_SET)) return 1;  // seek to section headers
	for (i = 0; i < file_hdr.e_shnum; i++)
	{
		if (fread(&sect_hdr, sizeof(sect_hdr), 1, f) != 1) { delete HeaderStringTable;  return 1; }

		if (sect_hdr.sh_type == SHT_PROGBITS && (sect_hdr.sh_flags & SHF_ALLOC) == SHF_ALLOC)
		{
			if (iLoadSections == MAX_SECTIONS)
			{
				MessageBoxW(NULL, Translate("Too many Sections in ELF File"), L"KMotion", MB_ICONSTOP|MB_OK|MB_TOPMOST|MB_SETFOREGROUND|MB_SYSTEMMODAL);
				return 1;
			}
			LoadSections[iLoadSections++] = sect_hdr;
		}
	}

	delete HeaderStringTable;
	
	// now load them all
	for (i = 0; i < iLoadSections; i++)
	{
		if (fseek(f, LoadSections[i].sh_offset, SEEK_SET)) return 1;  // seek to data

		// allocate memory for data
		unsigned char *buffer = (unsigned char *)malloc(LoadSections[i].sh_size);
		if (!buffer)
		{
			MessageBoxW(NULL, Translate("Memory Allocation Error for ELF File"), L"KMotion", MB_ICONSTOP|MB_OK|MB_TOPMOST|MB_SETFOREGROUND|MB_SYSTEMMODAL);
			fclose(f);
			return 1;
		}

		// read the data
		if (fread(buffer, LoadSections[i].sh_size, 1, f) != 1) {delete buffer; fclose(f); return 1; }

		// write it to DSP
		if (!mem_write(buffer, LoadSections[i].sh_size, LoadSections[i].sh_addr, 0)) { delete buffer; fclose(f); return 1; }

		delete buffer;
	}

	return 0;
}

void CKMotionDLL::DoErrMsg(const char* s)
{
	CStringW t = s;
	DoErrMsg(t);
}


void CKMotionDLL::DoErrMsg(const wchar_t *s)
{

	if (!ErrMessageDisplayed)
	{
		ErrMessageDisplayed=true;
		if (ErrMsgHandlerW)
		{
			__try
			{
				ErrMsgHandlerW(s);
			}
			__finally
			{
				ErrMessageDisplayed=false;
			}
		}
		else
		{
			MessageBoxW(NULL,s,L"KMotion",MB_ICONSTOP | MB_OK | MB_TOPMOST | MB_SETFOREGROUND | MB_SYSTEMMODAL);
		}
		ErrMessageDisplayed=false;
	}
}

int CKMotionDLL::GetStatus(MAIN_STATUS& status, bool lock)
{ 
    int word, i,result,n,token;
	CString s; 
	int *p=(int *)&status;

	n = sizeof(status) / sizeof(int);

	memset(p, 0, sizeof(status));  // clear all

	if (!ReadStatus)
	{
		if (!lock)ReleaseToken();  // was already locked?
		return 0;  // if wrong status version exit silently  
	}

	if(lock)
	{
		token = WaitToken( false, 100, "GetStatus");
		if (token != KMOTION_LOCKED) return 1;
	}

	// KMotion is available read the status
	s.Format("GetStatus");  
	if (WriteLine(s))
	{
		ReleaseToken();
		return 1;
	}


	s.Empty();

	for (i=0; i<n; i++)
	{
		if (s.IsEmpty())
		{
			if (ReadLineTimeOut(s.GetBuffer(2570),5000))  // big enough bufferfor  256 x 9 char hex strings 
			{
				ReleaseToken();
				return 1;
			}

			s.ReleaseBuffer();

			// change the CRLF at the to a space

			s.Delete(s.GetLength()-2,2);

			s += ' ';
		}

		// get a hex 32 bit int which may really be anything
		result = sscanf(s.GetBuffer(0),"%8X", &word);

		if (result!=1)
		{
			ReleaseToken();
			return 1;
		}

		if (ReadStatus)  // only fill structure if version matches
			*p++ = word;

		if (s.GetLength() >=9)
		{
			s.Delete(0,9);
		}
		else
		{
			ReleaseToken();
			return 1;
		}

		// check Version if first word
		if (i==0)
		{
			if (n != (status.VersionAndSize & 0xffff))
			{
				ReadStatus = false;

				// update number of words to read to avoid getting out of sync
				n = status.VersionAndSize & 0xffff;
			}
		}
	} 
	ReleaseToken(); 

	if (!ReadStatus)  // Wrong Version?
	{
		// note this may throw an exception and not return in .NET App
		// note this may throw an exception and not return in .NET App
		DoErrMsg(Translate("Error Status Record Size mismatch\r\rStatus updates disabled\r\rFlash Compatible Firmware and restart"));
	}

	return 0;
}


#define FLASHWRITER "\\DSP_KOGNA\\ti_tools\\flash_writer\\sfh_OMAP-L138.exe"
//c:\KMotionSrcKogna\DSP_KOGNA\ti_tools\flash_writer\sfh_OMAP-L138.exe -targettype C6748_LCDK -flashtype NAND -v -p COM10 -flash_noubl C:\KMotionSrcKogna\DSP_KOGNA\Debug\DSPKOGNA.bin
int CKMotionDLL::FlashKognaCOM(const char* Com)
{
	SECURITY_ATTRIBUTES sa = { 0 };
	STARTUPINFO         si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	HANDLE              hPipeOutputRead = NULL;
	HANDLE              hPipeOutputWrite = NULL;
	HANDLE              hPipeInputRead = NULL;
	HANDLE              hPipeInputWrite = NULL;
	BOOL                bTest = 0;
	DWORD               dwNumberOfBytesRead = 0;
	wchar_t             szMsg[100];

	CString Errors;



	// Try and locate the TI Flash Writer
	CString FlashWriter = MainPathRoot + FLASHWRITER;

	FILE *f = fopen(FlashWriter, "r");  // try rel where the KMotionDLL is

	if (f == NULL)
	{
		DoErrMsg(Translate("Error Locating ") + FLASHWRITER);
		return 1;
	}
	fclose(f);


	sa.nLength = sizeof(sa);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;

	// Make child process use hPipeOutputWrite as standard out,
	// and allow to show on screen.
	si.cb = sizeof(si);
	si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
	si.wShowWindow = SW_SHOW;
	si.hStdInput = NULL;
	si.hStdOutput = NULL;
	si.hStdError = hPipeOutputWrite;

	CString cmd;  // build command line
	CString BindTo, IncSrcPath1, IncSrcPath2, IncSrcPath3;

	cmd.Format(" -targettype C6748_LCDK -flashtype NAND -v -p %s -flash_noubl %s\\DSP_KOGNA\\DSPKOGNA.bin", Com, MainPathRoot);
	cmd = FlashWriter + cmd;

	CreateProcess(
		NULL,
		cmd.GetBuffer(0),
		NULL, NULL,
		TRUE, 0,
		NULL, NULL,
		&si, &pi);


	// Wait for CONSPAWN to finish.
	WaitForSingleObject(pi.hProcess, INFINITE);

	DWORD exitcode;
	int result = GetExitCodeProcess(pi.hProcess, &exitcode);

	// Now test to capture DOS application output by reading
	// hPipeOutputRead.  Could also write to DOS application
	// standard input by writing to hPipeInputWrite.

	if (exitcode != 0)
	{
		wsprintfW(szMsg, Translate("Error #%d Executing Flash over COM Port\r\rMake sure %s is present and not in use by another App"), GetLastError(), (CStringW)Com);
		DoErrMsg(szMsg);
		return 1;
	}
	return 0;
}

CStringW CKMotionDLL::Translate(CString s)
{
		return Trans.Translate(s);
}
