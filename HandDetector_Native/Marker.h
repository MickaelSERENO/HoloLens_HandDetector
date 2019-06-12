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

namespace Sereno
{
	/** Class representing a marker in the video stream.*/
	class Marker
	{
		public:
			/** Constructor
			 * \param eMarkerType the type of the marker
			 * \param pVarMarkerValue the marker value. This value is copied
			 * \param pVarContextValue the marker context. This value is copied*/
			Marker(MFSTREAMSINK_MARKER_TYPE eMarkerType, const PROPVARIANT* pVarMarkerValue , const PROPVARIANT* pVarContextValue) : m_eMarkerType(eMarkerType)
			{
				PropVariantCopy(&m_varMarkerValue, pVarMarkerValue);
				PropVariantCopy(&m_varContextValue, pVarContextValue);
			}

			MFSTREAMSINK_MARKER_TYPE getMarkerType() const { return m_eMarkerType; }
			void getMarkerValue(PROPVARIANT* out)    const { PropVariantCopy(out, &m_varMarkerValue); }
			void getContext(PROPVARIANT* out)        const { PropVariantCopy(out, &m_varContextValue); }
		protected:
			MFSTREAMSINK_MARKER_TYPE m_eMarkerType;
			PROPVARIANT m_varMarkerValue;
			PROPVARIANT m_varContextValue;
	};
}