/**
 * This file has no copyright assigned and is placed in the Public Domain.
 * This file is part of the mingw-w64 runtime package.
 * No warranty is given; refer to the file DISCLAIMER.PD within this package.
 */
#ifndef _DHCPSAPI_
#define _DHCPSAPI_

#ifndef WINAPI
#if defined(_ARM_)
#define WINAPI
#else
#define WINAPI __stdcall
#endif
#endif

#define DHCP_API_FUNCTION WINAPI

#ifdef __cplusplus
#define DHCP_CONST const
extern "C" {
#else
#define DHCP_CONST
#endif

typedef DWORD DHCP_IP_ADDRESS,*PDHCP_IP_ADDRESS,*LPDHCP_IP_ADDRESS;
typedef DWORD DHCP_IP_MASK;
typedef DWORD DHCP_RESUME_HANDLE;
typedef DWORD DHCP_OPTION_ID;

typedef enum _DHCP_OPTION_TYPE {
  DhcpUnaryElementTypeOption = 0,
  DhcpArrayTypeOption 
} DHCP_OPTION_TYPE, *LPDHCP_OPTION_TYPE;

typedef struct _DATE_TIME {
  DWORD dwLowDateTime;
  DWORD dwHighDateTime;
} DATE_TIME,*LPDATE_TIME;

typedef struct _DWORD_DWORD {
  DWORD DWord1;
  DWORD DWord2;
} DWORD_DWORD,*LPDWORD_DWORD;

typedef struct _DHCP_BINARY_DATA {
  DWORD DataLength;
  BYTE *Data;
} DHCP_BINARY_DATA,*LPDHCP_BINARY_DATA;

typedef DHCP_BINARY_DATA DHCP_CLIENT_UID;

#define DHCP_ENDPOINT_FLAG_CANT_MODIFY 0x01

typedef struct _DHCP_BIND_ELEMENT {
  ULONG Flags;
  WINBOOL fBoundToDHCPServer;
  DHCP_IP_ADDRESS AdapterPrimaryAddress;
  DHCP_IP_ADDRESS AdapterSubnetAddress;
  LPWSTR IfDescription;
  ULONG IfIdSize;
  LPBYTE IfId;
} DHCP_BIND_ELEMENT,*LPDHCP_BIND_ELEMENT;

typedef struct _DHCP_BIND_ELEMENT_ARRAY {
  DWORD NumElements;
  LPDHCP_BIND_ELEMENT Elements;
} DHCP_BIND_ELEMENT_ARRAY,*LPDHCP_BIND_ELEMENT_ARRAY;

typedef enum _DHCP_CLIENT_SEARCH_TYPE {
  DhcpClientIpAddress,DhcpClientHardwareAddress,DhcpClientName
} DHCP_SEARCH_INFO_TYPE,*LPDHCP_SEARCH_INFO_TYPE;

typedef struct _DHCP_CLIENT_SEARCH_INFO {
  DHCP_SEARCH_INFO_TYPE SearchType;
  union _DHCP_CLIENT_SEARCH_UNION {
    DHCP_IP_ADDRESS ClientIpAddress;
    DHCP_CLIENT_UID ClientHardwareAddress;
    LPWSTR ClientName;
  } SearchInfo;
} DHCP_SEARCH_INFO,*LPDHCP_SEARCH_INFO;

typedef enum _DHCP_OPTION_SCOPE_TYPE {
  DhcpDefaultOptions,DhcpGlobalOptions,DhcpSubnetOptions,DhcpReservedOptions,DhcpMScopeOptions
} DHCP_OPTION_SCOPE_TYPE,*LPDHCP_OPTION_SCOPE_TYPE;

typedef struct _DHCP_RESERVED_SCOPE {
  DHCP_IP_ADDRESS ReservedIpAddress;
  DHCP_IP_ADDRESS ReservedIpSubnetAddress;
} DHCP_RESERVED_SCOPE,*LPDHCP_RESERVED_SCOPE;

typedef struct _DHCP_OPTION_SCOPE_INFO {
  DHCP_OPTION_SCOPE_TYPE ScopeType;
  union _DHCP_OPTION_SCOPE_UNION {
    PVOID DefaultScopeInfo;
    PVOID GlobalScopeInfo;
    DHCP_IP_ADDRESS SubnetScopeInfo;
    DHCP_RESERVED_SCOPE ReservedScopeInfo;
    LPWSTR MScopeInfo;
  } ScopeInfo;
} DHCP_OPTION_SCOPE_INFO,*LPDHCP_OPTION_SCOPE_INFO;

typedef struct _DHCP_HOST_INFO {
  DHCP_IP_ADDRESS IpAddress;
  LPWSTR NetBiosName;
  LPWSTR HostName;
} DHCP_HOST_INFO,*LPDHCP_HOST_INFO;

typedef struct _DHCP_CLIENT_INFO {
  DHCP_IP_ADDRESS ClientIpAddress;
  DHCP_IP_MASK SubnetMask;
  DHCP_CLIENT_UID ClientHardwareAddress;
  LPWSTR ClientName;
  LPWSTR ClientComment;
  DATE_TIME ClientLeaseExpires;
  DHCP_HOST_INFO OwnerHost;
} DHCP_CLIENT_INFO,*LPDHCP_CLIENT_INFO;

typedef struct _DHCP_CLIENT_INFO_ARRAY {
  DWORD NumElements;
  LPDHCP_CLIENT_INFO *Clients;
} DHCP_CLIENT_INFO_ARRAY,*LPDHCP_CLIENT_INFO_ARRAY;

typedef struct _DHCP_IP_ARRAY {
  DWORD NumElements;
  LPDHCP_IP_ADDRESS Elements;
} DHCP_IP_ARRAY,*LPDHCP_IP_ARRAY;

typedef enum _DHCP_SUBNET_STATE {
  DhcpSubnetEnabled = 0,DhcpSubnetDisabled,DhcpSubnetEnabledSwitched,DhcpSubnetDisabledSwitched,DhcpSubnetInvalidState
} DHCP_SUBNET_STATE,*LPDHCP_SUBNET_STATE;

typedef struct _DHCP_SUBNET_INFO {
  DHCP_IP_ADDRESS SubnetAddress;
  DHCP_IP_MASK SubnetMask;
  LPWSTR SubnetName;
  LPWSTR SubnetComment;
  DHCP_HOST_INFO PrimaryHost;
  DHCP_SUBNET_STATE SubnetState;
} DHCP_SUBNET_INFO,*LPDHCP_SUBNET_INFO;

typedef enum _DHCP_OPTION_DATA_TYPE {
  DhcpByteOption,DhcpWordOption,DhcpDWordOption,DhcpDWordDWordOption,DhcpIpAddressOption,DhcpStringDataOption,DhcpBinaryDataOption,DhcpEncapsulatedDataOption
} DHCP_OPTION_DATA_TYPE,*LPDHCP_OPTION_DATA_TYPE;

typedef struct _DHCP_OPTION_DATA_ELEMENT {
  DHCP_OPTION_DATA_TYPE OptionType;
  union _DHCP_OPTION_ELEMENT_UNION {
    BYTE ByteOption;
    WORD WordOption;
    DWORD DWordOption;
    DWORD_DWORD DWordDWordOption;
    DHCP_IP_ADDRESS IpAddressOption;
    LPWSTR StringDataOption;
    DHCP_BINARY_DATA BinaryDataOption;
    DHCP_BINARY_DATA EncapsulatedDataOption;

  } Element;
} DHCP_OPTION_DATA_ELEMENT,*LPDHCP_OPTION_DATA_ELEMENT;

typedef struct _DHCP_OPTION_DATA {
  DWORD NumElements;
  LPDHCP_OPTION_DATA_ELEMENT Elements;
} DHCP_OPTION_DATA,*LPDHCP_OPTION_DATA;

typedef struct _DHCP_OPTION_VALUE {
  DHCP_OPTION_ID OptionID;
  DHCP_OPTION_DATA Value;
} DHCP_OPTION_VALUE,*LPDHCP_OPTION_VALUE;

typedef struct _DHCP_OPTION {
  DHCP_OPTION_ID   OptionID;
  LPWSTR           OptionName;
  LPWSTR           OptionComment;
  DHCP_OPTION_DATA DefaultValue;
  DHCP_OPTION_TYPE OptionType;
} DHCP_OPTION, *LPDHCP_OPTION;

DWORD WINAPI DhcpGetVersion(LPWSTR ServerIpAddress,LPDWORD MajorVersion,LPDWORD MinorVersion);
DWORD WINAPI DhcpSetServerBindingInfo(DHCP_CONST WCHAR *ServerIpAddress,ULONG Flags,LPDHCP_BIND_ELEMENT_ARRAY BindElementInfo);
DWORD WINAPI DhcpGetServerBindingInfo(DHCP_CONST WCHAR *ServerIpAddress,ULONG Flags,LPDHCP_BIND_ELEMENT_ARRAY *BindElementsInfo);
DWORD WINAPI DhcpCreateClientInfo(DHCP_CONST WCHAR *ServerIpAddress,DHCP_CONST DHCP_CLIENT_INFO *ClientInfo);
DWORD WINAPI DhcpSetClientInfo(DHCP_CONST WCHAR *ServerIpAddress,DHCP_CONST DHCP_CLIENT_INFO *ClientInfo);
DWORD WINAPI DhcpGetClientInfo(DHCP_CONST WCHAR *ServerIpAddress,DHCP_CONST DHCP_SEARCH_INFO *SearchInfo,LPDHCP_CLIENT_INFO *ClientInfo);
DWORD WINAPI DhcpDeleteClientInfo(DHCP_CONST WCHAR *ServerIpAddress,DHCP_CONST DHCP_SEARCH_INFO *ClientInfo);
DWORD WINAPI DhcpEnumSubnetClients(DHCP_CONST WCHAR *ServerIpAddress,DHCP_IP_ADDRESS SubnetAddress,DHCP_RESUME_HANDLE *ResumeHandle,DWORD PreferredMaximum,LPDHCP_CLIENT_INFO_ARRAY *ClientInfo,DWORD *ClientsRead,DWORD *ClientsTotal);
DWORD WINAPI DhcpEnumSubnets(DHCP_CONST WCHAR *ServerIpAddress,DHCP_RESUME_HANDLE *ResumeHandle,DWORD PreferredMaximum,LPDHCP_IP_ARRAY *EnumInfo,DWORD *ElementsRead,DWORD *ElementsTotal);
DWORD WINAPI DhcpGetSubnetInfo(DHCP_CONST WCHAR *ServerIpAddress,DHCP_IP_ADDRESS SubnetAddress,LPDHCP_SUBNET_INFO *SubnetInfo);
DWORD WINAPI DhcpGetOptionValue(DHCP_CONST WCHAR *ServerIpAddress,DHCP_OPTION_ID OptionID,DHCP_CONST DHCP_OPTION_SCOPE_INFO *ScopeInfo,LPDHCP_OPTION_VALUE *OptionValue);
VOID WINAPI DhcpRpcFreeMemory(PVOID BufferPointer);

#define ERROR_DHCP_REGISTRY_INIT_FAILED 20000
#define ERROR_DHCP_DATABASE_INIT_FAILED 20001
#define ERROR_DHCP_RPC_INIT_FAILED 20002
#define ERROR_DHCP_NETWORK_INIT_FAILED 20003
#define ERROR_DHCP_SUBNET_EXISTS 20004
#define ERROR_DHCP_SUBNET_NOT_PRESENT 20005
#define ERROR_DHCP_PRIMARY_NOT_FOUND 20006
#define ERROR_DHCP_ELEMENT_CANT_REMOVE 20007
#define ERROR_DHCP_OPTION_EXISTS 20009
#define ERROR_DHCP_OPTION_NOT_PRESENT 20010
#define ERROR_DHCP_ADDRESS_NOT_AVAILABLE 20011
#define ERROR_DHCP_RANGE_FULL 20012
#define ERROR_DHCP_JET_ERROR 20013
#define ERROR_DHCP_CLIENT_EXISTS 20014
#define ERROR_DHCP_INVALID_DHCP_MESSAGE 20015
#define ERROR_DHCP_INVALID_DHCP_CLIENT 20016
#define ERROR_DHCP_SERVICE_PAUSED 20017
#define ERROR_DHCP_NOT_RESERVED_CLIENT 20018
#define ERROR_DHCP_RESERVED_CLIENT 20019
#define ERROR_DHCP_RANGE_TOO_SMALL 20020
#define ERROR_DHCP_IPRANGE_EXISTS 20021
#define ERROR_DHCP_RESERVEDIP_EXISTS 20022
#define ERROR_DHCP_INVALID_RANGE 20023
#define ERROR_DHCP_RANGE_EXTENDED 20024
#define ERROR_DHCP_RANGE_EXTENSION_TOO_SMALL 20025
#define ERROR_DHCP_WARNING_RANGE_EXTENDED_LESS 20026
#define ERROR_DHCP_JET_CONV_REQUIRED 20027
#define ERROR_DHCP_SERVER_INVALID_BOOT_FILE_TABLE 20028
#define ERROR_DHCP_SERVER_UNKNOWN_BOOT_FILE_NAME 20029
#define ERROR_DHCP_SUPER_SCOPE_NAME_TOO_LONG 20030
#define ERROR_DHCP_IP_ADDRESS_IN_USE 20032
#define ERROR_DHCP_LOG_FILE_PATH_TOO_LONG 20033
#define ERROR_DHCP_UNSUPPORTED_CLIENT 20034
#define ERROR_DHCP_SERVER_INTERFACE_NOTIFICATION_EVENT 20035
#define ERROR_DHCP_JET97_CONV_REQUIRED 20036
#define ERROR_DHCP_ROGUE_INIT_FAILED 20037
#define ERROR_DHCP_ROGUE_SAMSHUTDOWN 20038
#define ERROR_DHCP_ROGUE_NOT_AUTHORIZED 20039
#define ERROR_DHCP_ROGUE_DS_UNREACHABLE 20040
#define ERROR_DHCP_ROGUE_DS_CONFLICT 20041
#define ERROR_DHCP_ROGUE_NOT_OUR_ENTERPRISE 20042
#define ERROR_DHCP_STANDALONE_IN_DS 20043
#define ERROR_DHCP_CLASS_NOT_FOUND 20044
#define ERROR_DHCP_CLASS_ALREADY_EXISTS 20045
#define ERROR_DHCP_SCOPE_NAME_TOO_LONG 20046
#define ERROR_DHCP_DEFAULT_SCOPE_EXISTS 20047
#define ERROR_DHCP_CANT_CHANGE_ATTRIBUTE 20048
#define ERROR_DHCP_IPRANGE_CONV_ILLEGAL 20049
#define ERROR_DHCP_NETWORK_CHANGED 20050
#define ERROR_DHCP_CANNOT_MODIFY_BINDINGS 20051
#define ERROR_DHCP_SUBNET_EXISTS 20052
#define ERROR_DHCP_MSCOPE_EXISTS 20053
#define ERROR_DHCP_MSCOPE_RANGE_TOO_SMALL 20054
#define ERROR_DHCP_MSCOPE_RANGE_TOO_SMALL 20054
#define ERROR_DDS_NO_DS_AVAILABLE 20070
#define ERROR_DDS_NO_DHCP_ROOT 20071
#define ERROR_DDS_DHCP_SERVER_NOT_FOUND 20074
#define ERROR_DDS_OPTION_ALREADY_EXISTS 20075
#define ERROR_DDS_OPTION_ALREADY_EXISTS 20076
#define ERROR_DDS_CLASS_EXISTS 20077
#define ERROR_DDS_CLASS_DOES_NOT_EXIST 20078
#define ERROR_DDS_SERVER_ALREADY_EXISTS 20079
#define ERROR_DDS_SERVER_DOES_NOT_EXIST 20080
#define ERROR_DDS_SERVER_ADDRESS_MISMATCH 20081
#define ERROR_DDS_SUBNET_EXISTS 20082
#define ERROR_DDS_SUBNET_HAS_DIFF_SUPER_SCOPE 20083
#define ERROR_DDS_SUBNET_NOT_PRESENT 20084
#define ERROR_DDS_RESERVATION_NOT_PRESENT 20085
#define ERROR_DDS_RESERVATION_CONFLICT 20086
#define ERROR_DDS_POSSIBLE_RANGE_CONFLICT 20087
#define ERROR_DDS_RANGE_DOES_NOT_EXIST 20088

typedef struct _DHCP_OPTION_ARRAY {
  DWORD         NumElements;
  LPDHCP_OPTION Options;
} DHCP_OPTION_ARRAY, *LPDHCP_OPTION_ARRAY;

#if (_WIN32_WINNT >= 0x0600)
typedef struct _DHCP_BINARY_DATA {
  DWORD DataLength;
  BYTE* Data;
} DHCP_BINARY_DATA, *LPDHCP_BINARY_DATA;

typedef DHCP_BINARY_DATA DHCP_CLIENT_UID; 

typedef enum _DHCP_OPTION_SCOPE_TYPE6 {
  DhcpDefaultOptions6,
  DhcpScopeOptions6,
  DhcpReservedOptions6 
} DHCP_OPTION_SCOPE_TYPE6;

typedef struct _DHCP_ALL_OPTIONS {
  DWORD               Flags;
  LPDHCP_OPTION_ARRAY NonVendorOptions;
  DWORD               NumVendorOptions;
  struct {
    DHCP_OPTION Option;
    LPWSTR      VendorName;
    LPWSTR      ClassName;
  } *VendorOptions;
} DHCP_ALL_OPTIONS, *LPDHCP_ALL_OPTIONS;

typedef struct _DHCP_IPV6_ADDRESS {
  ULONGLONG HighOrderBits;
  ULONGLONG LowOrderBits;
} DHCP_IPV6_ADDRESS, *PDHCP_IPV6_ADDRESS, *LPDHCP_IPV6_ADDRESS, DHCP_RESUME_IPV6_HANDLE;

typedef struct _DHCP_ALL_OPTION_VALUES {
  DWORD Flags;
  DWORD NumElements;
  struct {
    LPWSTR                    ClassName;
    LPWSTR                    VendorName;
    WINBOOL                   IsVendor;
    LPDHCP_OPTION_VALUE_ARRAY OptionsArray;
  } *Options;
} DHCP_ALL_OPTION_VALUES, *LPDHCP_ALL_OPTION_VALUES;

typedef struct _DHCP_OPTION_SCOPE_INFO6 {
  DHCP_OPTION_SCOPE_TYPE6 ScopeType;
  union {
    PVOID                DefaultScopeInfo;
    DHCP_IPV6_ADDRESS    SubnetScopeInfo;
    DHCP_RESERVED_SCOPE6 ReservedScopeInfo;
  } ScopeInfo;
} DHCP_OPTION_SCOPE_INFO6, *PDHCP_OPTION_SCOPE_INFO6, *LPDHCP_OPTION_SCOPE_INFO6;

typedef struct _DHCP_OPTION_VALUE_ARRAY {
  DWORD               NumElements;
  LPDHCP_OPTION_VALUE Values;
} DHCP_OPTION_VALUE_ARRAY, *LPDHCP_OPTION_VALUE_ARRAY;

typedef enum _DHCP_SUBNET_ELEMENT_TYPE_V6 {
  Dhcpv6IpRanges,
  Dhcpv6ReservedIps,
  Dhcpv6ExcludedIpRanges
} DHCP_SUBNET_ELEMENT_TYPE_V6, *LPDHCP_SUBNET_ELEMENT_TYPE_V6;

typedef struct _DHCP_IP_RANGE_V6 {
  DHCP_IPV6_ADDRESS StartAddress;
  DHCP_IPV6_ADDRESS EndAddress;
} DHCP_IP_RANGE_V6, *LPDHCP_IP_RANGE_V6;

typedef struct _DHCP_IP_RESERVATION_V6 {
  DHCP_IPV6_ADDRESS ReservedIpAddress;
  DHCP_CLIENT_UID* ReservedForClient;
  DWORD InterfaceId;
} DHCP_IP_RESERVATION_V6, *LPDHCP_IP_RESERVATION_V6;

typedef struct DHCP_SUBNET_ELEMENT_DATA_V6 {
  DHCP_SUBNET_ELEMENT_TYPE_V6 ElementType;
    union _DHCP_SUBNET_ELEMENT_UNION_V6 {
      DHCP_IP_RANGE_V6* IpRange;
      DHCP_IP_RESERVATION_V6* ReservedIp;
      DHCP_IP_RANGE_V6* ExcludeIpRange;
  } Element;
} DHCP_SUBNET_ELEMENT_DATA_V6, *LDHCP_SUBNET_ELEMENT_DATA_V6, *LPDHCP_SUBNET_ELEMENT_DATA_V6;

typedef struct _DHCP_SUBNET_ELEMENT_INFO_ARRAY_V6 {
  DWORD NumElements;
  LPDHCP_SUBNET_ELEMENT_DATA_V6 Elements;
} DHCP_SUBNET_ELEMENT_INFO_ARRAY_V6, *LPDHCP_SUBNET_ELEMENT_INFO_ARRAY_V6;

typedef struct _DHCP_SUBNET_INFO_V6 {
  DHCP_IPV6_ADDRESS SubnetAddress;
  ULONG             Prefix;
  USHORT            Preference;
  LPWSTR            SubnetName;
  LPWSTR            SubnetComment;
  DWORD             State;
  DWORD             ScopeId;
} DHCP_SUBNET_INFO_V6, *PDHCP_SUBNET_INFO_V6, *LPDHCP_SUBNET_INFO_V6;

DWORD DHCP_API_FUNCTION DhcpAddSubnetElementV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  LDHCP_SUBNET_ELEMENT_DATA_V6 *AddElementInfo
);

DWORD DHCP_API_FUNCTION DhcpCreateOptionV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  DHCP_OPTION_ID OptionId,
  WCHAR *ClassName,
  WCHAR *VendorName,
  LPDHCP_OPTION OptionInfo
);

DWORD DHCP_API_FUNCTION DhcpDeleteSubnetV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  DHCP_FORCE_FLAG ForceFlag
);

DWORD DHCP_API_FUNCTION DhcpCreateSubnetV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  LDHCP_SUBNET_INFO_V6 *SubnetInfo
);

DWORD DHCP_API_FUNCTION DhcpEnumOptionsV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  WCHAR *ClassName,
  WCHAR *VendorName,
  DHCP_RESUME_HANDLE *ResumeHandle,
  DWORD PreferredMaximum,
  LPDHCP_OPTION_ARRAY *Options,
  DWORD *OptionsRead,
  DWORD *OptionsTotal
);

DWORD DHCP_API_FUNCTION DhcpEnumOptionValuesV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  WCHAR *ClassName,
  WCHAR *VendorName,
  DHCP_OPTION_SCOPE_INFO6 ScopeInfo,
  DHCP_RESUME_HANDLE *ResumeHandle,
  DWORD PreferredMaximum,
  LPDHCP_OPTION_VALUE_ARRAY *OptionValues,
  DWORD *OptionsRead,
  DWORD *OptionsTotal
);

DWORD DHCP_API_FUNCTION DhcpEnumSubnetClientsV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  DHCP_RESUME_IPV6_HANDLE *ResumeHandle,
  DWORD PreferredMaximum,
  LPDHCP_CLIENT_INFO_ARRAY_V6 *ClientInfo,
  DWORD *ClientsRead,
  DWORD *ClientsTotal
);

DWORD DHCP_API_FUNCTION DhcpEnumSubnetElementsV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  DHCP_SUBNET_ELEMENT_TYPE_V6 EnumElementType,
  DHCP_RESUME_HANDLE *ResumeHandle,
  DWORD PreferredMaximum,
  LPDHCP_SUBNET_ELEMENT_INFO_ARRAY_V6 *EnumElementInfo,
  DWORD *ElementsRead,
  DWORD *ElementsTotal
);

DWORD DHCP_API_FUNCTION DhcpEnumSubnetsV6(
  LPWSTR ServerIpAddress,
  DHCP_RESUME_HANDLE *ResumeHandle,
  DWORD PreferredMaximum,
  LPDHCPV6_IP_ARRAY *EnumInfo,
  DWORD *ElementsRead,
  DWORD *ElementsTotal
);

DWORD DHCP_API_FUNCTION DhcpGetAllOptionsV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  LPDHCP_ALL_OPTIONS *OptionStruct
);

DWORD DHCP_API_FUNCTION DhcpGetAllOptionValuesV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  LPDHCP_OPTION_SCOPE_INFO6 ScopeInfo,
  LPDHCP_ALL_OPTION_VALUES *Values
);

DWORD DHCP_API_FUNCTION DhcpGetOptionInfoV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  DHCP_OPTION_ID OptionID,
  WCHAR *ClassName,
  WCHAR *VendorName,
  LPDHCP_OPTION *OptionInfo
);

DWORD DHCP_API_FUNCTION DhcpGetSubnetInfoV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  LPDHCP_SUBNET_INFO_V6 *SubnetInfo
);

DWORD DHCP_API_FUNCTION DhcpRemoveOptionV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  DHCP_OPTION_ID OptionID,
  WCHAR *ClassName,
  WCHAR *VendorName
);

DWORD DHCP_API_FUNCTION DhcpRemoveOptionValueV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  DHCP_OPTION_ID OptionID,
  WCHAR *ClassName,
  WCHAR *VendorName,
  DHCP_OPTION_SCOPE_INFO6 ScopeInfo
);

DWORD DHCP_API_FUNCTION DhcpRemoveSubnetElementV6(
  LPWSTR ServerIpAddress,
  DHCP_IPV6_ADDRESS SubnetAddress,
  LDHCP_SUBNET_ELEMENT_DATA_V6 RemoveElementInfo,
  DHCP_FORCE_FLAG ForceFlag
);

DWORD DHCP_API_FUNCTION DhcpSetOptionInfoV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  DHCP_OPTION_ID OptionID,
  WCHAR *ClassName,
  WCHAR *VendorName,
  LPDHCP_OPTION OptionInfo
);

DWORD DHCP_API_FUNCTION DhcpSetOptionValueV6(
  LPWSTR ServerIpAddress,
  DWORD Flags,
  DHCP_OPTION_ID OptionId,
  WCHAR *ClassName,
  WCHAR *VendorName,
  LDHCP_OPTION_SCOPE_INFO6 ScopeInfo,
  LDHCP_OPTION_DATA OptionValue
);

#endif /*(_WIN32_WINNT >= 0x0600)*/

#ifdef __cplusplus
}
#endif
#endif
