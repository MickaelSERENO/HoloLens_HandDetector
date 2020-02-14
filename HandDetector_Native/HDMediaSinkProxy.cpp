#include "HDMediaSinkProxy.h"
#include "HDMediaSink.h"

using namespace Sereno;

namespace HandDetector_Native
{
	HDMediaSinkProxy::HDMediaSinkProxy()
	{
		TRACE(L"In constructor\n")
	}

	HDMediaSinkProxy::~HDMediaSinkProxy()
	{
		TRACE(L"In destructor\n")
		AutoLock lock(m_critSec);

		if(m_spMediaSink != nullptr)
		{
			m_spMediaSink->Shutdown();
			m_spMediaSink = nullptr;
		}
	}

	Windows::Media::IMediaExtension^ HDMediaSinkProxy::GetMFExtensions()
	{
		AutoLock lock(m_critSec);

		if (m_spMediaSink == nullptr)
			Throw(MF_E_NOT_INITIALIZED)

		Microsoft::WRL::ComPtr<IInspectable> spInspectable;
		ThrowIfError(m_spMediaSink.As(&spInspectable))

		return safe_cast<Windows::Media::IMediaExtension^>(reinterpret_cast<Object^>(spInspectable.Get()));
	}

	Windows::Foundation::IAsyncOperation<Windows::Media::IMediaExtension^>^ HDMediaSinkProxy::InitializeAsync(IHDMediaSinkClbk^ clbk, Windows::Media::MediaProperties::IMediaEncodingProperties^ videoEncodingProperties)
	{
		return concurrency::create_async([this, clbk, videoEncodingProperties]()
		{
			AutoLock lock(m_critSec);

			if (m_spMediaSink != nullptr)
				Throw(MF_E_ALREADY_INITIALIZED)

			// Prepare the MF extension
			ThrowIfError(Microsoft::WRL::MakeAndInitialize<HDMediaSink>(&m_spMediaSink, clbk, videoEncodingProperties))

			Microsoft::WRL::ComPtr<IInspectable> spInspectable;
			ThrowIfError(m_spMediaSink.As(&spInspectable));
			
			return safe_cast<Windows::Media::IMediaExtension^>(reinterpret_cast<Object^>(spInspectable.Get()));
		});
	}
}