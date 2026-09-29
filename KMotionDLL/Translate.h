#pragma once

// This class is exported from the GCodeInterpreter.dll


class  CTranslate {
public:
	CTranslate();
	CString Translate(CString s);

	bool CheckedForList;
	bool ListLoaded;

	CList<CString, CString> EnglishList;
	CList<CString, CString> TanslateList;

};

extern CTranslate Trans;  // global instance

CString Translate(CString s);
