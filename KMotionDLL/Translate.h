#pragma once

// This class is exported from the GCodeInterpreter.dll


class  CTranslate {
public:
	CTranslate();
	CStringW Translate(CString s);
	CStringW Translate(char*);

	bool CheckedForList;
	bool ListLoaded;

	CList<CStringW, CStringW> EnglishList;
	CList<CStringW, CStringW> TanslateList;

};

extern CTranslate Trans;  // global instance

CStringW Translate(CString s);
