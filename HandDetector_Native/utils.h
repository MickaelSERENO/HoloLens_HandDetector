#pragma once

#include <strsafe.h>
#include <cstdlib>

#include <wrl\client.h>
#include <wrl\implements.h>
#include <wrl\ftm.h>
#include <wrl\event.h> 
#include <wrl\wrappers\corewrappers.h>


/** Check the result of a HRESULT function*/
#define CHECK(x) \
	{\
		HRESULT _hr = (x);\
		if(!SUCCEEDED(_hr))\
		{\
			TRACE(L"HRESULT : %d", _hr)\
			return _hr;\
		}\
	}

/** Get the filename being compiled*/
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__))

/** Debug trace on screen*/
#define TRACE(format, ...) \
	{\
		WCHAR _debugBuf[1024]; \
		WCHAR _fileName[1024]; \
		mbstowcs_s(NULL, _fileName, __FILENAME__, strlen(__FILENAME__));\
		\
		StringCchPrintf(_debugBuf, sizeof(_debugBuf)/sizeof(_debugBuf[0]), L"%s:%d " format, _fileName, __LINE__, __VA_ARGS__);\
		OutputDebugString(_debugBuf);\
	}

//#undef TRACE
//#define TRACE(format, ...) {}


/** Throw an exception based on hr*/
#define Throw(hr)\
	{\
		assert(FAILED(hr));\
		throw ref new Platform::Exception(hr);\
	}

/** Throw an exception if hr is a failure*/
#define ThrowIfError(hr)\
	{\
		if(FAILED(hr))\
			throw ref new Platform::Exception(hr);\
	}