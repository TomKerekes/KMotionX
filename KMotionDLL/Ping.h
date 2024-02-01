#pragma once

int FindKognas();
int FindKFLOPs();

#define MAX_KOGNAS 16
#define MAX_KFLOPS 16

typedef struct
{
    unsigned long KognaIP;
    unsigned long AdapterIP;
    int KognaSerialNumber;
} KOGNA_INFO;


extern int nKognas;
extern KOGNA_INFO Kognas[MAX_KOGNAS];  // Kogna Online list
extern volatile bool FirstKognasScanComplete;

extern DWORD nKFLOPs;
extern FT_DEVICE_LIST_INFO_NODE KFLOPs[MAX_KFLOPS];  // KFLOP Online list

extern HANDLE KognaListMutex;
extern HANDLE KFLOPListMutex;
