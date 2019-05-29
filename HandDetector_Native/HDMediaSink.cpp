#include "HDMediaSink.h"

namespace Sereno
{
	static HRESULT addAttribute(GUID guidKey, Windows::Foundation::IPropertyValue ^value, IMFAttributes *pAttr)
	{
		Windows::Foundation::PropertyType type = value->Type;
		switch(type)
		{
			case Windows::Foundation::PropertyType::UInt8Array:
			{
				Platform::Array<BYTE>^ arr;
				value->GetUInt8Array(&arr);
				CHECK(pAttr->SetBlob(guidKey, arr->Data, arr->Length))
				break;
			}

			case Windows::Foundation::PropertyType::Double:
				CHECK(pAttr->SetDouble(guidKey, value->GetDouble()))
				break;

			case Windows::Foundation::PropertyType::Guid:
				CHECK(pAttr->SetGUID(guidKey, value->GetGuid()))
				break;

			case Windows::Foundation::PropertyType::String:
				CHECK(pAttr->SetString(guidKey, value->GetString()->Data()))
				break;

			case Windows::Foundation::PropertyType::UInt32:
				CHECK(pAttr->SetUINT32(guidKey, value->GetUInt32()))
				break;

			case Windows::Foundation::PropertyType::UInt64:
				CHECK(pAttr->SetUINT64(guidKey, value->GetUInt64()))
				break;
		}
		return S_OK;
	}

	HRESULT convertPropertiesToMediaType(Windows::Media::MediaProperties::IMediaEncodingProperties^ mep, IMFMediaType **ppMT)
	{
		TRACE(L"In convert properties\n")
		if (mep == nullptr || ppMT == nullptr)
			return E_INVALIDARG;

		Microsoft::WRL::ComPtr<IMFMediaType> spMT;
		*ppMT = nullptr;
		CHECK(MFCreateMediaType(&spMT))

		for(auto it = mep->Properties->First(); it->HasCurrent; it->MoveNext())
		{
			auto currentValue = it->Current;
			addAttribute(currentValue->Key, safe_cast<Windows::Foundation::IPropertyValue^>(currentValue->Value), spMT.Get());
		}

		GUID guiMajorType = safe_cast<Windows::Foundation::IPropertyValue^>(mep->Properties->Lookup(MF_MT_MAJOR_TYPE))->GetGuid();
		if (guiMajorType != MFMediaType_Video)
			return MF_E_INVALIDMEDIATYPE;
		*ppMT = spMT.Detach();

		return S_OK;
	}

	HDMediaSink::HDMediaSink()
	{}

	HDMediaSink::~HDMediaSink()
	{}

	HRESULT HDMediaSink::RuntimeClassInitialize(HandDetector_Native::IHDMediaSinkClbk^ clbk, Windows::Media::MediaProperties::IMediaEncodingProperties ^videoEncodingProperties)
	{
		TRACE(L"In runtime class initialize\n")

		if(videoEncodingProperties != nullptr)
		{
			CHECK(convertPropertiesToMediaType(videoEncodingProperties, &m_mediaType))
			GUID majorType, subType;
			CHECK(m_mediaType->GetGUID(MF_MT_MAJOR_TYPE, &majorType))
			CHECK(m_mediaType->GetGUID(MF_MT_SUBTYPE, &subType))


			TRACE(L"MajorType : %ws\n", majorType == MFMediaType_Video ? L"Video" : L"Unknown")
			TRACE(L"Finish converting\n")
			if(majorType != MFMediaType_Video || (subType != MFVideoFormat_RGB24 && subType != MFVideoFormat_L8 && subType != MFVideoFormat_D16 && subType != MFVideoFormat_L16 && subType != MFVideoFormat_ARGB32))
				return MF_E_INVALIDMEDIATYPE;

			m_stream.Attach(new HDStreamSink());
			m_stream->Initialize(this, clbk, SERENO_STREAM_ID_VIDEO);
			m_stream->SetCurrentMediaType(m_mediaType.Get());
		}
		else
			return MF_E_INVALIDMEDIATYPE;
		m_clbk = clbk;
		TRACE(L"End Runtime class initialize\n")
		return S_OK;
	}


	HRESULT HDMediaSink::GetCharacteristics(DWORD *pdwCharacteristics)
	{
		TRACE(L"In GetCharacteristics\n")
		AutoLock lock(m_critSec);
		*pdwCharacteristics = MEDIASINK_RATELESS | MEDIASINK_FIXED_STREAMS;
		return S_OK;
	}

	HRESULT HDMediaSink::AddStreamSink(DWORD dwStreamSinkIdentifier, IMFMediaType *pMediaType, IMFStreamSink **ppStreamSink)
	{
		return MF_E_STREAMSINKS_FIXED;
	}

	HRESULT HDMediaSink::RemoveStreamSink(DWORD dwStreamSinkIdentifier)
	{
		return MF_E_STREAMSINKS_FIXED;
	}

	HRESULT HDMediaSink::GetStreamSinkCount(_Out_ DWORD *pcStreamSinkCount)
	{
		TRACE(L"In GetStreamSinkCount\n")
		AutoLock lock(m_critSec);
		*pcStreamSinkCount = 1;
		return S_OK;
	}

	HRESULT HDMediaSink::GetStreamSinkByIndex(DWORD dwIndex, IMFStreamSink **ppStreamSink)
	{
		TRACE(L"In GetStreamSinkByIndex. Asking index : %d\n", (int)dwIndex)

		AutoLock lock(m_critSec);
		if(dwIndex > 0)
			return MF_E_INVALIDINDEX;

		TRACE(L"Correct index\n")
		*ppStreamSink = m_stream.Get();
		if(m_stream.Get())
			m_stream->AddRef();
		return S_OK;
	}

	HRESULT HDMediaSink::GetStreamSinkById(DWORD dwId, IMFStreamSink **ppStreamSink)
	{
		TRACE(L"In GetStreamSinkByIndex. Asking index : %d\n", (int)dwId)

		AutoLock lock(m_critSec);
		if(dwId != SERENO_STREAM_ID_VIDEO)
			return MF_E_INVALIDINDEX;

		TRACE("Correct index\n")
		*ppStreamSink = m_stream.Get();
		if(m_stream.Get())
			m_stream->AddRef();
		return S_OK;
	}

	HRESULT HDMediaSink::SetPresentationClock(IMFPresentationClock *pPresentationClock)
	{
		TRACE(L"In SetPresentationClock\n")
		AutoLock lock(m_critSec);
		if(m_spClock != NULL)
			CHECK(m_spClock->RemoveClockStateSink(this))

		m_spClock = pPresentationClock;
		CHECK(m_spClock->AddClockStateSink(this))
		return S_OK;
	}

	HRESULT HDMediaSink::GetPresentationClock(IMFPresentationClock **ppPresentationClock)
	{
		TRACE(L"In GetPresentationClock\n")

		AutoLock lock(m_critSec);
		*ppPresentationClock = m_spClock.Get();
		if(*ppPresentationClock == NULL)
			return MF_E_NO_CLOCK;
		(*ppPresentationClock)->AddRef();

		return S_OK;
	}

	HRESULT HDMediaSink::Shutdown()
	{
		TRACE(L"In Shutdown\n")

		AutoLock lock(m_critSec);
		if(m_isShutdown)
			return MF_E_SHUTDOWN;

		if(m_stream.Get())
			CHECK(m_stream->Shutdown())
		m_isShutdown = true;
		return S_OK;
	}

	HRESULT HDMediaSink::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
	{
		TRACE(L"In OnClockStart\n")

		return m_stream->Start(hnsSystemTime);
	}

	HRESULT HDMediaSink::OnClockStop(MFTIME hnsSystemTime)
	{
		TRACE(L"In OnClockStop\n")

		return m_stream->Stop();
	}

	HRESULT HDMediaSink::OnClockPause(MFTIME hnsSystemTime)
	{
		TRACE(L"In OnClockPause\n")

		return m_stream->Pause();
	}

	HRESULT HDMediaSink::OnClockRestart(MFTIME hnsSystemTime)
	{
		TRACE(L"In OnClockRestart\n")

		return m_stream->Restart();
	}
	HRESULT HDMediaSink::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
	{
		TRACE(L"In OnClockSetRate\n")

		return S_OK;
	}
}