#include <KMotionX.h>
#include "KMotionDLL.h"
#include "Translate.h"

// Linux version of Translate.cpp. Strings are UTF-8 (narrow) here; the translation file is
// read as wide text, like Windows does, and converted.

CTranslate Trans;

CString Translate(CString s)
{
	return Trans.Translate(s);
}

CTranslate::CTranslate() : CheckedForList(false), ListLoaded(false) {}

CString CTranslate::Translate(CString s)
{
    if (CheckedForList && !ListLoaded) return s;

    if (!CheckedForList) {
        LoadTranslationList();
    }

    if (ListLoaded) {
        // look for a match
        for (size_t i = 0; i < EnglishList.size(); ++i) {
            if (s == EnglishList[i]) {
                return CString(TranslateList[i]);
            }
        }
    }

    return s;
}

void CTranslate::LoadTranslationList() {
    wchar_t wcsString[4001];

    CheckedForList = true;

    // Open the file with the specified encoding
    // Some editors like NotePad++ don't put BOM Byte Order Mark so better to let Windows decide
    std::string filePath = kmx::getLocalLanguageFilePath();
    //log_info("reading translationfile: %s", filePath.c_str());
    std::wifstream fileStream(filePath, std::ios::binary);

    if (!fileStream.is_open()) {
        return;
    }

    std::wstring sRead, Eng, Trans;

    while (fileStream.getline(wcsString, 4000)) {
        sRead = wcsString;

        // Remove trailing carriage return if present
        if (!sRead.empty() && sRead.back() == L'\r') {
            sRead.pop_back();
        }

        if (!sRead.empty()) {
            // file format is English Left ..#.. Translated Right
            size_t i = sRead.find(L"    ..#..    ");

            if (i != std::wstring::npos) {
                Eng = sRead.substr(0, i);
                Trans = sRead.substr(i + 13);

                EnglishList.push_back(kmx::wstrtostr(Eng));
                TranslateList.push_back(kmx::wstrtostr(Trans));
            }
        }
    }

    ListLoaded = true;
}
