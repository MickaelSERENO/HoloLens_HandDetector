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
#include <windows.networking.sockets.h>

#include <wrl.h>
#include <wrl\client.h>
#include <wrl\implements.h>
#include <wrl\ftm.h>
#include <wrl\event.h> 
#include <wrl\wrappers\corewrappers.h>

#include <ppltasks.h>

#include "CritSec.h"
#include "HandDetectorProxy.h"
#include "IHDMediaSinkClbk.h"

using namespace Sereno;

namespace HandDetector_Native
{
	/** Proxy class creating the object HDMediaSink*/
	public ref class HDMediaSinkProxy sealed
	{
		public:
			/** Constructor, does nothing. Call InitializeAsync to initialize this Object*/
			HDMediaSinkProxy();

			virtual ~HDMediaSinkProxy();

			/** Get the MediaFoundation Extension created by this class proxy
			  * \return the IMediaExtension class which is in fact a HDMediaSink object*/
			Windows::Media::IMediaExtension^ GetMFExtensions();

			/** \briefAsynchronous initialization of the MediaSink used for RTP streaming
			  * \param clbk the interface class to call when the media sink status changes
			  * \param videoEncodingProperties the selected video properties to use for the moment*/
			Windows::Foundation::IAsyncOperation<Windows::Media::IMediaExtension^>^ InitializeAsync(IHDMediaSinkClbk^ clbk, Windows::Media::MediaProperties::IMediaEncodingProperties^ videoEncodingProperties);
		private:
			CritSec m_critSec;                                  /*!< Permits to define a critical section to lock*/
			Microsoft::WRL::ComPtr<IMFMediaSink> m_spMediaSink; /*!< The HDMediaSink object created by this proxy class*/
	};
}
