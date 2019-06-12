#pragma once

#include <windows.h>
#include <mfidl.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfobjects.h>

#include <windows.foundation.h>
#include <windows.foundation.collections.h>
#include <windows.media.h>
#include <windows.media.capture.h>
#include <windows.media.mediaproperties.h>

#include <wrl.h>
#include <wrl\client.h>
#include <wrl\implements.h>
#include <wrl\ftm.h>
#include <wrl\event.h> 
#include <wrl\wrappers\corewrappers.h>

#include "BaseAttribute.h"
#include "HDStreamSink.h"
#include "CritSec.h"
#include "utils.h"

#define SERENO_STREAM_ID_VIDEO 0

namespace Sereno
{
	/** \brief Class handling media (like device media streams). This is the first MediaFoundation class starting the parsing pipeline*/
	class HDMediaSink : public Microsoft::WRL::RuntimeClass<Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::RuntimeClassType::WinRtClassicComMix>,
					 									      ABI::Windows::Media::IMediaExtension, IMFMediaSink, IMFClockStateSink>, public CBaseAttributes<>
	{
		InspectableClass(L"Sereno::HDMediaSink", BaseTrust)

		public:
			HDMediaSink();
			virtual ~HDMediaSink();

			/** WRL function permiting to initialize this object.
			 * \param clbk the callback interface to call when the detection status changed
			 * \param videoEncodingProperties the initialize video encoding properties to use*/
			HRESULT RuntimeClassInitialize(HandDetector_Native::IHDMediaSinkClbk^ clbk, Windows::Media::MediaProperties::IMediaEncodingProperties ^videoEncodingProperties);

			/////////////////////
			// IMediaExtension //
			/////////////////////

			/** \brief Initialize the video sink via properties.
			 * \param pConfiguration The properties set*/
			STDMETHODIMP SetProperties(ABI::Windows::Foundation::Collections::IPropertySet *pConfiguration) { return S_OK; }

			//////////////////
			// IMFMediaSink //
			//////////////////

			/** \brief Get the characteristics of this sink.
			 * \param pdwCharacteristics[out] the characteristics of the sink. This sink allow only one and only one stream*/
			STDMETHODIMP GetCharacteristics(DWORD *pdwCharacteristics);

			/** \brief Add a new stream sink to this media (audio, video). Only Video is handled (SERENO_STREAM_ID_VIDEO)
			 * \param[in] dwStreamSinkIdentifier the stream sink identifier. 
			 * \param[in] pMediaType the stream sink media type
			 * \param[out] The stream sink created.*/
			STDMETHODIMP AddStreamSink(DWORD dwStreamSinkIdentifier, IMFMediaType *pMediaType, IMFStreamSink **ppStreamSink);

			/** \brief Remove a Stream sink
			 * \param dwStreamSinkIdentifier The stream sink identifier to remove. Only Video is handled (SERENO_STREAM_ID_VIDEO)*/
			STDMETHODIMP RemoveStreamSink(DWORD dwStreamSinkIdentifier);

			/** \brief Get the stream sink count registered.
			 * \param[out] pcStreamSinkCount the number of stream sink actually registered. This is set automatically at 1 (only the video sink)*/
			STDMETHODIMP GetStreamSinkCount(DWORD *pcStreamSinkCount);
			
			STDMETHODIMP GetStreamSinkByIndex(DWORD dwIndex, IMFStreamSink **ppStreamSink);
			STDMETHODIMP GetStreamSinkById(DWORD dwIdentifier, IMFStreamSink **ppStreamSink);
			STDMETHODIMP SetPresentationClock(IMFPresentationClock *pPresentationClock);
			STDMETHODIMP GetPresentationClock(IMFPresentationClock **ppPresentationClock);
			STDMETHODIMP Shutdown();

			// IMFClockStateSink methods
			STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
			STDMETHODIMP OnClockStop(MFTIME hnsSystemTime);
			STDMETHODIMP OnClockPause(MFTIME hnsSystemTime);
			STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime);
			STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate);

		private:
			CritSec  m_critSec;            /*!<critical section for thread safety*/
			bool     m_isShutdown = false; /*!<Flag to indicate if Shutdown() method was called.*/

			Microsoft::WRL::ComPtr<IMFMediaType>         m_mediaType; /*!< The default media type asked*/
			Microsoft::WRL::ComPtr<HDStreamSink>         m_stream;    /*!< The HDStreamSink object acquiring and processing the depth images*/
			Microsoft::WRL::ComPtr<IMFPresentationClock> m_spClock;   /*!< Presentation clock.*/
			HandDetector_Native::IHDMediaSinkClbk^       m_clbk;      /*!< The callback interface to call when the detection status changes*/
	};
}