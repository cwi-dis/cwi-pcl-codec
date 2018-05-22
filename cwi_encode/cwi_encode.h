// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the CWI_ENCODE_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// CWI_ENCODE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef CWI_ENCODE_EXPORTS
#define CWI_ENCODE_API __declspec(dllexport)
#else
#define CWI_ENCODE_API __declspec(dllimport)
#endif

// This class is exported from the cwi_encode.dll
class CWI_ENCODE_API Ccwi_encode {
public:
	Ccwi_encode(void);
	// TODO: add your methods here.
};

extern CWI_ENCODE_API int ncwi_encode;

CWI_ENCODE_API int fncwi_encode(void);
