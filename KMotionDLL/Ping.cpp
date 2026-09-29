#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "stdafx.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <icmpapi.h>
#include <stdio.h>
#include "Ping.h"

#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")

DWORD ScanAdapters(LPDWORD lpdwParam);
int ScanAdaptersErr();
DWORD ReceiveResponses(LPDWORD lpdwParam);
int JoinMulticastGroup(SOCKET s, struct addrinfo* group, struct addrinfo* iface);
int SetSendInterface(SOCKET s, struct addrinfo* iface);
int SetMulticastTtl(SOCKET s, int af, int ttl);
struct addrinfo* ResolveAddress(const char* addr, const char* port, int af, int type, int proto);
int FindIPAddr(unsigned long* IP, unsigned long* Subnet, unsigned short* Type, int max_addr, int* nfound);
DWORD ScanKFLOPs(LPDWORD lpdwParam);
static int SendKognaQuery(unsigned long AdapterIP, int* pWSAError);
static void RecordScanError(int Step, int WSAError, unsigned long AdapterIP);
static void ReportScanFailure();

HANDLE KognaListMutex;
HANDLE KFLOPListMutex;

#define MAX_ADAPTERS 16      // max adapters scanned (after filtering)
#define MAX_IP_ADDRS 32      // max IPv4 addresses read from the system
#define BUF_SIZE 64

// Number of consecutive scan passes in which EVERY adapter failed before the
// User is told about it.  Individual/transient failures (an adapter dropping
// link, Wi-Fi roaming, a VPN reconnecting, DHCP renewing, ...) are expected
// and are simply retried on the next pass.
#define SCAN_FAILURES_BEFORE_ALERT 10

// GetIpAddrTable wType flags (ipmib.h) - guard in case of older SDK headers
#ifndef MIB_IPADDR_DISCONNECTED
#define MIB_IPADDR_DISCONNECTED 0x0008
#endif
#ifndef MIB_IPADDR_DELETED
#define MIB_IPADDR_DELETED 0x0040
#endif

int nKognas = 0;
KOGNA_INFO Kognas[MAX_KOGNAS];  // Adapter List
bool volatile FirstKognasScanComplete = false;

// Diagnostics describing the most recent scan failure.  Visible in the
// debugger, in DebugView (OutputDebugString), and in the alert message.
int LastScanError = 0;                  // step code (-2..-25) that failed
int LastScanWSAError = 0;               // WSAGetLastError()/GetLastError() at that time
unsigned long LastScanErrorAdapterIP = 0; // adapter (network byte order) that failed
int ConsecutiveFailedScans = 0;         // passes in a row where every adapter failed
static bool ScanFailureReported = false; // alert displayed at most once per process


DWORD nKFLOPs;
FT_DEVICE_LIST_INFO_NODE KFLOPs[MAX_KFLOPS];  // KFLOP Online list


typedef struct
{
    int n;
    unsigned long KognaIP[MAX_KOGNAS];
    unsigned long AdapterIP;
    int KognaSerialNumber[MAX_KOGNAS];
    volatile int result;    // receive thread: -1 = still running, 0 = OK, < -1 = error
    int recvWSAError;       // WSAGetLastError() if the receive thread failed
    int sendResult;         // 0 = query sent OK, else step code that failed
} INVOKE_PARAMS;

static INVOKE_PARAMS params[MAX_ADAPTERS];  // Adapter List
unsigned long IPs[MAX_IP_ADDRS], Subnet[MAX_IP_ADDRS];
unsigned short IPTypes[MAX_IP_ADDRS];
int nfound;

WSADATA             wsd;

int FindKognas()
{
    // Load Winsock
    if (WSAStartup(MAKEWORD(1, 1), &wsd) != 0) return -1;

    // Create a mutex with no initial owner
    KognaListMutex = CreateMutex(
        NULL,              // default security attributes
        FALSE,             // initially not owned
        NULL);             // unnamed mutex

    // create a worker Thread to scan for adapters in system
    HANDLE Thread = CreateThread(
        NULL,                        /* no security attributes        */
        100000,                      /* stack size 100K        */
        (LPTHREAD_START_ROUTINE) ::ScanAdapters, /* thread function       */
        NULL,	    			     /* argument to thread function   */
        0,                           /* use default creation flags    */
        NULL);

    if (Thread == NULL) return -1;
    CloseHandle(Thread);  // thread runs for the life of the process; handle not needed
    return 0;
}

int FindKFLOPs()
{
    // Create a mutex with no initial owner
    KFLOPListMutex = CreateMutex(
        NULL,              // default security attributes
        FALSE,             // initially not owned
        NULL);             // unnamed mutex

    // create a worker Thread to scan for adapters in system
    HANDLE Thread = CreateThread(
        NULL,                        /* no security attributes        */
        100000,                      /* stack size 100K        */
        (LPTHREAD_START_ROUTINE) ::ScanKFLOPs, /* thread function       */
        NULL,	    			     /* argument to thread function   */
        0,                           /* use default creation flags    */
        NULL);

    if (Thread == NULL) return -1;
    CloseHandle(Thread);  // thread runs for the life of the process; handle not needed
    return 0;
}

DWORD ScanKFLOPs(LPDWORD lpdwParam)
{
    FT_STATUS ftStatus;
    DWORD numDevs;

    for (;;)
    {
        ftStatus = FT_CreateDeviceInfoList(&numDevs);

        if (ftStatus == FT_OK)
        {
            DWORD dwWaitResult = WaitForSingleObject(KFLOPListMutex, INFINITE);  // no time-out interval
            if (dwWaitResult == WAIT_OBJECT_0)
            {
                if (numDevs > 0)
                {
                    ftStatus = FT_GetDeviceInfoList(KFLOPs, &numDevs);

                    nKFLOPs = 0;
                    for (int i = 0; i < (int)numDevs; i++)
                    {
                        if (strstr(KFLOPs[i].Description, "KFLOP") != NULL ||
                            strstr(KFLOPs[i].Description, "KMotion") != NULL ||
                            strstr(KFLOPs[i].Description, "Dynomotion") != NULL)
                        {
                            KFLOPs[nKFLOPs++] = KFLOPs[i];
                        }
                    }
                }
                else
                {
                    nKFLOPs = 0;
                }
                ReleaseMutex(KFLOPListMutex);
            }
        }

        Sleep(1000);
    }
    return 0;
}


// Remember the details of a scan failure and trace it for diagnostics
static void RecordScanError(int Step, int WSAError, unsigned long AdapterIP)
{
    char s[128];

    LastScanError = Step;
    LastScanWSAError = WSAError;
    LastScanErrorAdapterIP = AdapterIP;

    sprintf(s, "KMotionServer: Kogna scan step %d failed on adapter %d.%d.%d.%d (Windows error %d)\n",
        Step, AdapterIP & 0xff, (AdapterIP >> 8) & 0xff, (AdapterIP >> 16) & 0xff, (AdapterIP >> 24) & 0xff, WSAError);
    OutputDebugStringA(s);
}

// Tell the User (once per process) that the scan has been failing persistently
static void ReportScanFailure()
{
    if (ScanFailureReported) return;
    ScanFailureReported = true;

    CString s;
    s.Format(Translate("Scanning for Kogna's Failed - Error %d (Windows Error %d) on Network Adapter %d.%d.%d.%d\r\rKFLOP USB operation is unaffected.  If no Kogna is used the scan may be disabled by starting with the -no_ethernet option."),
        LastScanError, LastScanWSAError,
        LastScanErrorAdapterIP & 0xff, (LastScanErrorAdapterIP >> 8) & 0xff,
        (LastScanErrorAdapterIP >> 16) & 0xff, (LastScanErrorAdapterIP >> 24) & 0xff);
    MessageBoxW(NULL, s, L"KMotion", MB_ICONEXCLAMATION | MB_OK | MB_TOPMOST | MB_SETFOREGROUND);
}


DWORD ScanAdapters(LPDWORD lpdwParam)
{
    for (;;)  // if some fatal error, try again
    {
        int result = ScanAdaptersErr();   // only returns on a fatal (non-adapter) error
        if (result)
        {
            RecordScanError(result, GetLastError(), 0);
            ConsecutiveFailedScans++;
            if (ConsecutiveFailedScans >= SCAN_FAILURES_BEFORE_ALERT) ReportScanFailure();

            // a fatal error might mean Winsock is in a bad state - restart it
            WSACleanup();
            if (WSAStartup(MAKEWORD(1, 1), &wsd) != 0)
                MessageBoxW(NULL, Translate("Winsock Startup Failed"), L"KMotion", MB_ICONSTOP|MB_OK|MB_TOPMOST|MB_SETFOREGROUND|MB_SYSTEMMODAL);
        }
        if (nKognas > 0)  // tktk after a Kogna has been found update infrequently for testing
            Sleep(10000);
        else
            Sleep(1000);
    }
    return 0;
}

// Continuously scan all local network adapters for Kognas.
//
// Failures on an individual adapter are expected from time to time (an
// adapter dropping link to save power, Wi-Fi roaming, a VPN reconnecting,
// DHCP renewing, virtual adapters coming and going, ...).  Such an adapter
// is simply skipped for this pass and retried on the next one; the other
// adapters are still scanned and the pass still completes so that
// FirstKognasScanComplete is set and Apps (including KFLOP Apps) are not
// blocked waiting for it.  Only returns on a fatal error.
int ScanAdaptersErr()
{
    int i;

    for (;;)
    {
        if (FindIPAddr(IPs, Subnet, IPTypes, MAX_IP_ADDRS, &nfound)) return 1;

        // filter non-local adapters out, and adapters that Windows reports
        // as disconnected (no link) or being removed - joining a multicast
        // group on those fails with WSAEADDRNOTAVAIL
        int k = 0;
        for (i = 0; i < nfound && k < MAX_ADAPTERS; i++)
        {
            if (IPTypes[i] & (MIB_IPADDR_DISCONNECTED | MIB_IPADDR_DELETED)) continue;

            if ((IPs[i] & 0xff) == 192 || (IPs[i] & 0xff) == 172 || (IPs[i] & 0xff) == 10)
            {
                params[k].AdapterIP = IPs[i];
                params[k].result = 0;    // nothing running yet
                params[k].sendResult = 0;
                params[k].recvWSAError = 0;
                params[k].n = 0;
                k++;
            }
        }
        nfound = k;


        for (i = 0; i < nfound; i++)
        {
            // create a worker Thread to handle responses
            params[i].result = -1;  // receive thread running
            HANDLE Thread = CreateThread(
                NULL,                        /* no security attributes        */
                100000,                      /* stack size 100K        */
                (LPTHREAD_START_ROUTINE) ::ReceiveResponses, /* thread function       */
                &params[i],	    			 /* argument to thread function   */
                0,                           /* use default creation flags    */
                NULL);

            if (Thread == NULL)
            {
                params[i].result = -13;
                params[i].recvWSAError = GetLastError();
                RecordScanError(-13, params[i].recvWSAError, params[i].AdapterIP);
                continue;  // skip this adapter for this pass
            }
            CloseHandle(Thread);  // completion is signaled via params[i].result

            // multicast the query on this adapter
            int WSAError;
            params[i].sendResult = SendKognaQuery(params[i].AdapterIP, &WSAError);
            if (params[i].sendResult)
                RecordScanError(params[i].sendResult, WSAError, params[i].AdapterIP);
            // (the receive thread simply times out if nothing was sent)
        }

        if (nfound == 0) Sleep(250);  // avoid tight loop when nothing found

        // wait for all adapters to finish
        int nAdaptersFailed = 0;
        for (i = 0; i < nfound; i++)
        {
            while (params[i].result == -1)
                Sleep(10);

            if (params[i].result != 0)  // receive thread failed
                RecordScanError(params[i].result, params[i].recvWSAError, params[i].AdapterIP);

            if (params[i].result != 0 || params[i].sendResult != 0)
                nAdaptersFailed++;
        }

        // process every Kogna found
        DWORD dwWaitResult = WaitForSingleObject(KognaListMutex, INFINITE);  // no time-out interval
        if (dwWaitResult != WAIT_OBJECT_0) return -12;

        int n = 0;
        for (i = 0; i < nfound; i++)  // loop through Adapters
        {
            for (k = 0; k < params[i].n && n < MAX_KOGNAS; k++)
            {
                // put it in the list
                Kognas[n].KognaIP = ntohl(params[i].KognaIP[k]);
                Kognas[n].AdapterIP = ntohl(params[i].AdapterIP);
                Kognas[n].KognaSerialNumber = params[i].KognaSerialNumber[k];
                n++;
            }
        }
        nKognas = n;

        ReleaseMutex(KognaListMutex);

        FirstKognasScanComplete = true;

        // Only consider the pass failed if every adapter failed.  Tell the
        // User once if that keeps happening.
        if (nfound > 0 && nAdaptersFailed == nfound)
        {
            ConsecutiveFailedScans++;
            if (ConsecutiveFailedScans >= SCAN_FAILURES_BEFORE_ALERT) ReportScanFailure();
        }
        else
        {
            ConsecutiveFailedScans = 0;
        }

        if (nKognas > 0) // after one found slow down multicasting
            Sleep(10000);
        else
            Sleep(1000);

    }
}


// Send one multicast "Kogna?" query out of the specified adapter.
// Returns 0 on success, otherwise the (negative) step that failed with
// *pWSAError set to WSAGetLastError().  All resources are released.
static int SendKognaQuery(unsigned long AdapterIP, int* pWSAError)
{
    char buf[BUF_SIZE];
    int result = 0;
    SOCKET s = INVALID_SOCKET;
    struct addrinfo* resmulti = NULL, * resbind = NULL, * resif = NULL;

    const char* gBindAddr = NULL,   // Address to bind socket to (default is 0.0.0.0 or ::)
        * gMulticast = "239.81.92.240",   // Multicast group to join
        * gPort = "25000";          // Port number to use

    int   gSocketType = SOCK_DGRAM,   // datagram
        gProtocol = IPPROTO_UDP,    // UDP
        gTtl = 8;         // Multicast TTL value

    char gInterface[40];          // Interface to join the multicast group on

    *pWSAError = 0;

    sprintf(gInterface, "%d.%d.%d.%d", AdapterIP & 0xff, (AdapterIP >> 8) & 0xff,
        (AdapterIP >> 16) & 0xff, (AdapterIP >> 24) & 0xff);

    do
    {
        // Resolve the multicast address
        resmulti = ResolveAddress(gMulticast, gPort, AF_UNSPEC, gSocketType, gProtocol);
        if (resmulti == NULL) { result = -2; break; }

        // Resolve the binding address
        resbind = ResolveAddress(gBindAddr, "0", resmulti->ai_family, resmulti->ai_socktype, resmulti->ai_protocol);
        if (resbind == NULL) { result = -3; break; }

        // Resolve the multicast interface
        resif = ResolveAddress(gInterface, "0", resmulti->ai_family, resmulti->ai_socktype, resmulti->ai_protocol);
        if (resif == NULL) { result = -4; break; }

        // Create the socket. In Winsock 1 you don't need any special
        // flags to indicate multicasting.
        s = socket(resmulti->ai_family, resmulti->ai_socktype, resmulti->ai_protocol);
        if (s == INVALID_SOCKET) { result = -5; break; }

        // Bind the socket to the local interface. This is done so we can receive data
        int rc = bind(s, resbind->ai_addr, (int)resbind->ai_addrlen);
        if (rc == SOCKET_ERROR) { result = -6; break; }

        // Join the multicast group.  This is not required to *send* to the
        // group (responses arrive by unicast on port 25001) so a failure
        // here (typically WSAEADDRNOTAVAIL while an adapter is changing
        // state) is only traced and the query is still attempted.
        rc = JoinMulticastGroup(s, resmulti, resif);
        if (rc == SOCKET_ERROR)
        {
            char t[128];
            sprintf(t, "KMotionServer: Kogna scan multicast join failed on adapter %s (Windows error %d) - continuing\n",
                gInterface, WSAGetLastError());
            OutputDebugStringA(t);
        }

        // Set the send (outgoing) interface
        rc = SetSendInterface(s, resif);
        if (rc == SOCKET_ERROR) { result = -8; break; }

        // Set the TTL to something else. The default TTL is one.
        rc = SetMulticastTtl(s, resmulti->ai_family, gTtl);
        if (rc == SOCKET_ERROR) { result = -9; break; }

        memset(buf, 0, BUF_SIZE);
        sprintf(buf, "Kogna?");

        // Send some data
        rc = sendto(s, buf, BUF_SIZE, 0, resmulti->ai_addr, (int)resmulti->ai_addrlen);
        if (rc == SOCKET_ERROR) { result = -11; break; }
    } while (0);

    if (result) *pWSAError = WSAGetLastError();

    if (s != INVALID_SOCKET) closesocket(s);
    if (resif) freeaddrinfo(resif);
    if (resbind) freeaddrinfo(resbind);
    if (resmulti) freeaddrinfo(resmulti);

    return result;
}


// Receive thread: listens on port 25001 of the adapter for "I am Kogna SNxxx"
// replies until a 0.5 second timeout.  Always sets p->result on exit
// (0 = OK, < -1 = failure with p->recvWSAError) so the scanner never waits
// forever.
DWORD ReceiveResponses(LPDWORD lpdwParam)
{
    INVOKE_PARAMS *p = (INVOKE_PARAMS*)lpdwParam;
    int result = 0;

    sockaddr_in socketAddress = { 0 };
    socketAddress.sin_family = PF_INET;
    socketAddress.sin_port = htons(25001);
    socketAddress.sin_addr.s_addr = p->AdapterIP;

    // Create the socket
    SOCKET mSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (mSocket == INVALID_SOCKET)
    {
        p->recvWSAError = WSAGetLastError();
        p->result = -21;
        return 0;
    }

    do
    {
        int enable = 1;
        if (setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0) { result = -22; break; }

        //Bind
        if (bind(mSocket, (struct sockaddr*)&socketAddress, sizeof(socketAddress)) == SOCKET_ERROR) { result = -23; break; }

        double timeout = 0.5;

        // Set timeout
        DWORD lBuffer[2] = { 0, 0 };
        int lSize;
        lBuffer[0] = static_cast<DWORD>(1000.0 * timeout);
        lSize = sizeof(DWORD);
        if (setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)lBuffer, lSize) != 0) { result = -24; break; }

        // Check that we get what we set.
        DWORD lBufferout[2] = { 0, 0 };
        if (getsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)lBufferout, &lSize) != 0) { result = -25; break; }

        // Receive and time
        char buffer[BUF_SIZE + 1];
        sockaddr_in senderAddr;
        int senderAddrSize;

        for (int k = 0; k < MAX_KOGNAS && p->n < MAX_KOGNAS; k++)  // assume there may be multiple responses
        {
            senderAddrSize = sizeof(senderAddr);
            int transferred = recvfrom(mSocket, buffer, BUF_SIZE, 0,
                (sockaddr*)&senderAddr, &senderAddrSize);

            if (transferred == SOCKET_ERROR) break;  // timeout (or error) - done

            if (transferred == BUF_SIZE)
            {
                int SerialNumber;
                buffer[BUF_SIZE] = 0;  // guarantee termination
                int r = sscanf(buffer + 13, "%d", &SerialNumber);
                buffer[13] = 0;  // null terminate
                if (r == 1 && strcmp("I am Kogna SN", buffer) == 0)
                {
                    p->KognaSerialNumber[p->n] = SerialNumber;
                    p->KognaIP[p->n] = senderAddr.sin_addr.S_un.S_addr;
                    p->n++;
                }
            }
        }
    } while (0);

    if (result) p->recvWSAError = WSAGetLastError();

    shutdown(mSocket, SD_SEND);
    closesocket(mSocket);

    p->result = result;  // flag Thread finished.

    return 0;
}


// Function: JoinMulticastGroup
// Description:
//    This function joins the multicast socket on the specified multicast
//    group. The structures for IPv4 and IPv6 multicast joins are slightly
//    different which requires different handlers. For IPv6 the scope-ID
//    (interface index) is specified for the local interface whereas for IPv4
//    the actual IPv4 address of the interface is given.
int JoinMulticastGroup(SOCKET s, struct addrinfo* group, struct addrinfo* iface)
{
    struct ip_mreq   mreqv4;
    char* optval = NULL;
    int    optlevel, option, optlen, rc;

    rc = NO_ERROR;
    // Setup the v4 option values and ip_mreq structure
    optlevel = IPPROTO_IP;
    option = IP_ADD_MEMBERSHIP;
    optval = (char*)&mreqv4;
    optlen = sizeof(mreqv4);

    mreqv4.imr_multiaddr.s_addr = ((SOCKADDR_IN*)group->ai_addr)->sin_addr.s_addr;
    mreqv4.imr_interface.s_addr = ((SOCKADDR_IN*)iface->ai_addr)->sin_addr.s_addr;

    if (rc != SOCKET_ERROR)
    {
        // Join the group
        rc = setsockopt(s, optlevel, option, optval, optlen);
    }
    return rc;
}

// Function: SetSendInterface
// Description:
//    This routine sets the send (outgoing) interface of the socket.
//    Again, for v4 the IP address is used to specify the interface while
//    for v6 its the scope-ID.
int SetSendInterface(SOCKET s, struct addrinfo* iface)
{
    char* optval = NULL;
    int   optlevel, option, optlen, rc;

    rc = NO_ERROR;

    // Setup the v4 option values
    optlevel = IPPROTO_IP;
    option = IP_MULTICAST_IF;
    optval = (char*)&((SOCKADDR_IN*)iface->ai_addr)->sin_addr.s_addr;
    optlen = sizeof(((SOCKADDR_IN*)iface->ai_addr)->sin_addr.s_addr);

    // Set send IF
    if (rc != SOCKET_ERROR)
    {
        // Set the send interface
        rc = setsockopt(s, optlevel, option, optval, optlen);
    }
    return rc;
}

// Function: SetMulticastTtl
// Description: This routine sets the multicast TTL value for the socket.
int SetMulticastTtl(SOCKET s, int af, int ttl)
{
    char* optval = NULL;
    int   optlevel, option, optlen, rc;

    // Set the options for V4
    optlevel = IPPROTO_IP;
    option = IP_MULTICAST_TTL;
    optval = (char*)&ttl;
    optlen = sizeof(ttl);
    // Set the TTL value
    rc = setsockopt(s, optlevel, option, optval, optlen);
    return rc;
}

#define MALLOC(x) HeapAlloc(GetProcessHeap(), 0, (x))
#define FREE(x) HeapFree(GetProcessHeap(), 0, (x))


// Returns the IPv4 addresses in the system along with their subnet masks and
// the MIB_IPADDR_xxx type flags (so callers can skip disconnected adapters).
int FindIPAddr(unsigned long* IP, unsigned long* Subnet, unsigned short* Type, int max_addr, int* nfound)
{

    int i;

    /* Variables used by GetIpAddrTable */
    PMIB_IPADDRTABLE pIPAddrTable;
    DWORD dwSize = 0;
    DWORD dwRetVal = 0;


    // Before calling AddIPAddress we use GetIpAddrTable to get
    // an adapter to which we can add the IP.
    pIPAddrTable = (MIB_IPADDRTABLE*)MALLOC(sizeof(MIB_IPADDRTABLE));
    if (pIPAddrTable == NULL) { SetLastError(ERROR_NOT_ENOUGH_MEMORY); return 1; }

    // Make an initial call to GetIpAddrTable to get the
    // necessary size into the dwSize variable
    if (GetIpAddrTable(pIPAddrTable, &dwSize, 0) == ERROR_INSUFFICIENT_BUFFER)
    {
        FREE(pIPAddrTable);
        pIPAddrTable = (MIB_IPADDRTABLE*)MALLOC(dwSize);
    }
    if (pIPAddrTable == NULL) { SetLastError(ERROR_NOT_ENOUGH_MEMORY); return 1; }

    // Make a second call to GetIpAddrTable to get the
    // actual data we want
    if ((dwRetVal = GetIpAddrTable(pIPAddrTable, &dwSize, 0)) != NO_ERROR)
    {
        FREE(pIPAddrTable);
        SetLastError(dwRetVal);  // so the caller can report why
        return 1;
    }

    *nfound = (int)(pIPAddrTable->dwNumEntries);
    if (*nfound > max_addr) *nfound = max_addr;
    for (i = 0; i < *nfound; i++)
    {
        IP[i] = (u_long)pIPAddrTable->table[i].dwAddr;
        Subnet[i] = (u_long)pIPAddrTable->table[i].dwMask;
        Type[i] = pIPAddrTable->table[i].wType;
    }

    if (pIPAddrTable) {
        FREE(pIPAddrTable);
        pIPAddrTable = NULL;
    }

    return 0;
}


// Function: ResolveAddress
// Description:
//    This routine resolves the specified address and returns a list of addrinfo
//    structure containing SOCKADDR structures representing the resolved addresses.
//    Note that if 'addr' is non-NULL, then getaddrinfo will resolve it whether
//    it is a string literal address or a hostname.
struct addrinfo* ResolveAddress(const char* addr, const char* port, int af, int type, int proto)
{
    struct addrinfo hints, * res = NULL;
    int             rc;

    memset(&hints, 0, sizeof(hints));
    hints.ai_flags = ((addr) ? 0 : AI_PASSIVE);
    hints.ai_family = af;
    hints.ai_socktype = type;
    hints.ai_protocol = proto;

    rc = getaddrinfo(addr, port, &hints, &res);
    if (rc != 0) return NULL;
    return res;
}
