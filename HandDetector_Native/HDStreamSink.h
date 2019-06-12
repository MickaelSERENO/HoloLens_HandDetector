#pragma once

#include <windows.h>
#include <mfidl.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfobjects.h>
#include <mfmediacapture.h>
#include <mfmediaengine.h>

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

#include <queue>
#include <memory>

#include "utils.h"
#include "CritSec.h"
#include "Marker.h"
#include "StreamOperations.h"
#include "AsyncOperation.h"
#include "SampleData.h"
#include "AsyncCallback.h"
#include "HandDetector.h"
#include "IHDMediaSinkClbk.h"

#if !WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
EXTERN_GUID(MFSampleExtension_Spatial_CameraCoordinateSystem, 0x9d13c82f, 0x2199, 0x4e67, 0x91, 0xcd, 0xd1, 0xa4, 0x18, 0x1f, 0x25, 0x34);
EXTERN_GUID(MFSampleExtension_Spatial_CameraViewTransform, 0x4e251fa4, 0x830f, 0x4770, 0x85, 0x9a, 0x4b, 0x8d, 0x99, 0xaa, 0x80, 0x9b);
EXTERN_GUID(MFSampleExtension_Spatial_CameraProjectionTransform, 0x47f9fcb5, 0x2a02, 0x4f26, 0xa4, 0x77, 0x79, 0x2f, 0xdf, 0x95, 0x88, 0x6a);
#endif

namespace Sereno
{
	/** \brief Network Sink. This object takes in entry frames encoded in H264 and stream out a RTP stream using ffmpeg*/
	class HDStreamSink : public IMFStreamSink, public IMFMediaTypeHandler
	{
		public:
			/** \brief Matrix permitting to tell which operation are possible according to a status (pause, stop, etc.)*/
			static BOOL validStateMatrix[State_Count][Op_Count];
		public:
			/** \brief Constructor. Call Initialize to initialize this object*/
			HDStreamSink();

			~HDStreamSink();

			/** \brief Initialize the internal state of this stream sink. 
			 * \param parent the parent MediaSink this stream sink is link to		
			 * \param clbk the callback interface to call when the detection status changed
			 * \param identifier the sink identifier*/
			HRESULT Initialize(IMFMediaSink* parent, HandDetector_Native::IHDMediaSinkClbk^ clbk, DWORD identifier);
			
			// IUnknown
			IFACEMETHOD(QueryInterface) (REFIID riid, void **ppv);
			IFACEMETHOD_(ULONG, AddRef) ();
			IFACEMETHOD_(ULONG, Release) ();

			// IMFMediaEventGenerator
			IFACEMETHOD(BeginGetEvent)(IMFAsyncCallback *pCallback, IUnknown *punkState);
			IFACEMETHOD(EndGetEvent) (IMFAsyncResult *pResult, IMFMediaEvent **ppEvent);
			IFACEMETHOD(GetEvent) (DWORD dwFlags, IMFMediaEvent **ppEvent);
			IFACEMETHOD(QueueEvent) (MediaEventType met, REFGUID guidExtendedType, HRESULT hrStatus, PROPVARIANT const *pvValue);

			// IMFStreamSink
			IFACEMETHOD(GetMediaSink) (IMFMediaSink **ppMediaSink);
			IFACEMETHOD(GetIdentifier) (DWORD *pdwIdentifier);
			IFACEMETHOD(GetMediaTypeHandler) (IMFMediaTypeHandler **ppHandler);
			IFACEMETHOD(ProcessSample) (IMFSample *pSample);
			IFACEMETHOD(PlaceMarker) (MFSTREAMSINK_MARKER_TYPE eMarkerType, PROPVARIANT const *pvarMarkerValue, PROPVARIANT const *pvarContextValue);
			IFACEMETHOD(Flush)();

			// IMFMediaTypeHandler

			/** Test if the media type pMediaType is supported or not.
			 * This class support type = VIDEO and subtype = RGB24, L8, L16 and D16
			 
			 * \param pMediaType the media type to test
			 * \param ppMediaType[out] the closest match media type supported. This function sets this parameter to NULL*/
			IFACEMETHOD(IsMediaTypeSupported) (IMFMediaType *pMediaType, IMFMediaType **ppMediaType);

			IFACEMETHOD(GetMediaTypeCount) (DWORD *pdwTypeCount);
			IFACEMETHOD(GetMediaTypeByIndex) (DWORD dwIndex, IMFMediaType **ppType);
			IFACEMETHOD(SetCurrentMediaType) (IMFMediaType *pMediaType);
			IFACEMETHOD(GetCurrentMediaType) (IMFMediaType **ppMediaType);
			IFACEMETHOD(GetMajorType) (GUID *pguidMajorType);

			/** \brief Tells wheter or not the operation "op" is valid or not taking account the current status of the sink
			 * \param op the operation to evaluate*/
			BOOL ValidateOperation(StreamOperation op) const;

			/** Start this Sink.*/
			HRESULT     Start(MFTIME start);

			/** Restart this Sink*/
			HRESULT     Restart();

			/** Stop this Sink*/
			HRESULT     Stop();
			HRESULT     Pause();
			HRESULT     Shutdown();
		private:
			/** Asynchronously queue a new operation to treat. The caller must set what ever "queue" data accordingly (here m_queueData)
			 * \param op the new operation type.*/
			HRESULT QueueAsyncOperation(StreamOperation op);

			/** Function called in a separate thread by the Queue (m_workQueueCB and m_workQueueId) when a new operation has been queued.
			 * \param pAsyncResult the operation data*/
			HRESULT OnDispatchWorkItem(IMFAsyncResult *pAsyncResult);

			/** update the hand detection when a new frame arrived
			 * \param sample the image sample data
			 * \param rawBuffer the raw image buffer
			 * \param bufferCurrentLength the raw image buffer length*/
			HRESULT HDStreamSink::UpdateHandDetection(IMFSample* sample, BYTE* rawBuffer, unsigned long bufferCurrentLength);

			/** Process the format (MediaType) changes
			 * \param pMediaType the new media type*/
			void ProcessFormatChange(IMFMediaType *pMediaType);

			/** Write raw sample buffer data to a PPM file.
			 * \param rawBuffer the image buffer
			 * \param bufferCurrentLength the image buffer length*/
			HRESULT WriteSampleToFile(BYTE* rawBuffer, unsigned long bufferCurrentLength);

			ULONG m_cRef = 1; /** !< Variable used by IUnknown to know the current counter reference of this object*/

			Microsoft::WRL::ComPtr<IMFMediaEventQueue>  m_spEventQueue;  /** !< Event queue.  All action (pause, start, asking new frame) are done through this object*/
			Microsoft::WRL::ComPtr<IMFMediaType>        m_spCurrentType; /** !< The current media type applied*/
			Microsoft::WRL::ComPtr<IMFMediaSink>        m_spSink;        /** !< The Parent media sink*/
			Microsoft::WRL::ComPtr<IUnknown>            m_spFTM;         /** !< The FreeThreadedMarshaler. See CoCreateFreeThreadedMarshaler*/

			uint32_t m_streamWidth  = 0; /*!< The stream width in pixels*/
			uint32_t m_streamHeight = 0; /*!< The stream height in pixels*/

			GUID    m_mediaSubtype = MFVideoFormat_RGB24; /*!< The current media subtype. Handled : RGB24 and L8*/
			State   m_state = State_TypeNotSet; /*!< Current state of the sink*/
			CritSec m_critSec;                  /*!< Defines a critical section. We use this object as locker for thread safety*/
			
			DWORD                          m_workQueueId; /*!< ID of the work queue for asynchronous operations.*/
			AsyncCallback<HDStreamSink>    m_workQueueCB; /*!< Callback for the work queue. This object is used for asynchronous work*/
			std::queue<SampleData>         m_queueData;   /*!< Queue containing all the frame*/

			DWORD    m_identifier = 0; /*!< The Stream identifier defined by the MediaSink parent*/
			uint32_t m_frameID    = 0; /*!< The current frame ID*/

			IHandDetection* m_handDetector = NULL; /*!< The Hand Detector object computing the new hands position*/

			HandDetector_Native::IHDMediaSinkClbk^ m_clbk; /*!< The callback interface to call when the detection status changes*/
	};
}