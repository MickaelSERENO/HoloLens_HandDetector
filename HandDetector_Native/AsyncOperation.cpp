#include "AsyncOperation.h"

namespace Sereno
{
	AsyncOperation::AsyncOperation(StreamOperation o) : op(o), m_cRef(1)
	{}

	ULONG AsyncOperation::Release()
	{
		
		ULONG cRef = InterlockedDecrement(&m_cRef);
		if (cRef == 0)
			delete this;
		return cRef;
	}

	ULONG AsyncOperation::AddRef()
	{
		return InterlockedIncrement(&m_cRef);
	}

	HRESULT AsyncOperation::QueryInterface(REFIID iid, void **ppv)
	{
		if (!ppv)
			return E_POINTER;

		if (iid == IID_IUnknown)
			*ppv = static_cast<IUnknown*>(this);

		else
		{
			*ppv = nullptr;
			return E_NOINTERFACE;
		}
		AddRef();
		return S_OK;
	}
}