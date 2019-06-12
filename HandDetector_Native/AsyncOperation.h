#pragma once

#include <wrl.h>
#include <Windows.h>

#include "StreamOperations.h"

namespace Sereno
{
	/** Class containing the type of the operation being treated */
	class AsyncOperation : public IUnknown
	{
	public:
		AsyncOperation(StreamOperation o);
		StreamOperation op;   // The operation to perform.

		// IUnknown methods.
		STDMETHODIMP QueryInterface(REFIID iid, void **ppv);
		STDMETHODIMP_(ULONG) AddRef();
		STDMETHODIMP_(ULONG) Release();
	private:
		long    m_cRef = 1;
	};
}