#include "HDStreamSink.h"

#include <string>

#include "ICameraIntrinsics.h"

namespace Sereno
{
	// Class-static matrix of operations vs states.
	// If an entry is TRUE, the operation is valid from that state.
	BOOL HDStreamSink::validStateMatrix[State_Count][Op_Count] =
	{
		// States:    Operations:
		//            SetType   Start     Restart   Pause     Stop      Sample    Marker   
		/* NotSet */  TRUE,     FALSE,    FALSE,    FALSE,    FALSE,    FALSE,    FALSE,
		/* Ready */   TRUE,     TRUE,     FALSE,    TRUE,     TRUE,     FALSE,    TRUE,
		/* Start */   TRUE,     TRUE,     FALSE,    TRUE,     TRUE,     TRUE,     TRUE,
		/* Pause */   TRUE,     TRUE,     TRUE,     TRUE,     TRUE,     TRUE,     TRUE,
		/* Stop */    TRUE,     TRUE,     FALSE,    FALSE,    TRUE,     FALSE,    TRUE,
	};

	HDStreamSink::HDStreamSink() : m_workQueueCB(this, &HDStreamSink::OnDispatchWorkItem)
	{
		TRACE(L"In Constructor\n")
	}

	HDStreamSink::~HDStreamSink()
	{
		TRACE(L"In Destructor\n");
	}

	IFACEMETHODIMP_(ULONG) HDStreamSink::AddRef()
	{
		TRACE(L"In AddRef\n")

		return InterlockedIncrement(&m_cRef);
	}

	IFACEMETHODIMP_(ULONG) HDStreamSink::Release()
	{
		TRACE(L"In Release\n")

		long cRef = InterlockedDecrement(&m_cRef);
		if (cRef == 0)
		{
			delete this;
		}
		return cRef;
	}

	HRESULT HDStreamSink::QueryInterface(REFIID riid, void **ppv)
	{
		TRACE(L"In QueryInterface\n")

		if (ppv == nullptr)
			return E_POINTER;

		(*ppv) = nullptr;
		HRESULT hr = S_OK;

		if(riid == IID_IUnknown || riid == IID_IMFStreamSink || riid == IID_IMFMediaEventGenerator)
		{
			(*ppv) = static_cast<IMFStreamSink*>(this);
			AddRef();
		}

		else if(riid == IID_IMFMediaTypeHandler)
		{
			(*ppv) = static_cast<IMFMediaTypeHandler*>(this);
			AddRef();
		}

		else
			hr = E_NOINTERFACE;

		if(FAILED(hr) && riid == IID_IMarshal)
		{
			if(m_spFTM == nullptr)
			{
				AutoLock lock(m_critSec);
				if(m_spFTM == nullptr)
					hr = CoCreateFreeThreadedMarshaler(static_cast<IMFStreamSink*>(this), &m_spFTM);
			}

			if(SUCCEEDED(hr))
			{
				if(m_spFTM == nullptr)
					hr = E_UNEXPECTED;
				else
					hr = m_spFTM.Get()->QueryInterface(riid, ppv);
			}
		}
		return hr;
	}

	HRESULT HDStreamSink::Initialize(IMFMediaSink* parent, HandDetector_Native::IHDMediaSinkClbk^ clbk, DWORD identifier)
	{
		TRACE(L"In Initialize\n")
		m_clbk = clbk;
		m_identifier = identifier;
		AutoLock lock(m_critSec);

		Microsoft::WRL::ComPtr<IMFMediaType> mediaType;

		CHECK(MFCreateEventQueue(&m_spEventQueue))
		m_spSink = parent;
		CHECK(MFAllocateSerialWorkQueue(MFASYNC_CALLBACK_QUEUE_STANDARD, &m_workQueueId))

		//Create the current media type
		CHECK(MFCreateMediaType(&mediaType))
		CHECK(mediaType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video))
		CHECK(mediaType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB24))

		CHECK(SetCurrentMediaType(mediaType.Get()))

		return S_OK;
	}

	HRESULT HDStreamSink::BeginGetEvent(IMFAsyncCallback *pCallback, IUnknown *punkState)
	{
		TRACE(L"In BeginGetEvent\n")

		AutoLock lock(m_critSec);

		CHECK(m_spEventQueue->BeginGetEvent(pCallback, punkState))

		return S_OK;
	}

	HRESULT HDStreamSink::EndGetEvent(IMFAsyncResult *pResult, IMFMediaEvent **ppEvent)
	{
		TRACE(L"In EndGetEvent\n")

		AutoLock lock(m_critSec);
		CHECK(m_spEventQueue->EndGetEvent(pResult, ppEvent))

		return S_OK;
	}

	HRESULT HDStreamSink::GetEvent(DWORD dwFlags, IMFMediaEvent **ppEvent)
	{
		TRACE(L"In GetEvent\n")

		// NOTE:
		// GetEvent can block indefinitely, so we don't hold the lock.
		// This requires some juggling with the event queue pointer.
		Microsoft::WRL::ComPtr<IMFMediaEventQueue> spQueue;
		{
			AutoLock lock(m_critSec);
			spQueue = m_spEventQueue;
		}
		// Now get the event.
		CHECK(spQueue->GetEvent(dwFlags, ppEvent))

		return S_OK;
	}

	HRESULT HDStreamSink::QueueEvent(MediaEventType met, REFGUID guidExtendedType, HRESULT hrStatus, PROPVARIANT const *pvValue)
	{
		TRACE(L"In QueueEvent\n")

		CHECK(m_spEventQueue->QueueEventParamVar(met, guidExtendedType, hrStatus, pvValue))

		return S_OK;
	}

	HRESULT HDStreamSink::GetIdentifier(DWORD *pdwIdentifier)
	{
		TRACE(L"In GetIdentifier\n")

		*pdwIdentifier = 0;
		return S_OK;
	}

	HRESULT HDStreamSink::GetMediaSink(IMFMediaSink **ppMediaSink)
	{
		TRACE(L"In GetMediaSink\n")

		AutoLock lock(m_critSec);

		return m_spSink->QueryInterface(IID_IMFMediaSink, (void**)ppMediaSink);
	}

	HRESULT HDStreamSink::GetMediaTypeHandler(IMFMediaTypeHandler **ppHandler)
	{
		TRACE(L"In GetMediaTypeHandler\n")

		AutoLock lock(m_critSec);

		return QueryInterface(IID_IMFMediaTypeHandler, (void**)ppHandler);
	}

	HRESULT HDStreamSink::ProcessSample(IMFSample *pSample)
	{
		TRACE(L"In ProcessSample\n")

		if(pSample == nullptr)
			return E_INVALIDARG;

		AutoLock lock(m_critSec);

		//Validate the operation.
		CHECK(ValidateOperation(OpProcessSample))

		SampleData sampleData(ST_SAMPLE);
		sampleData.asCOM = pSample;
		pSample->AddRef();
		m_queueData.push(sampleData);
		QueueAsyncOperation(OpProcessSample);

		//Ask new frame
		CHECK(QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, nullptr))
		return S_OK;
	}

	HRESULT HDStreamSink::PlaceMarker(MFSTREAMSINK_MARKER_TYPE eMarkerType, const PROPVARIANT *pvarMarkerValue, const PROPVARIANT *pvarContextValue)
	{
		TRACE(L"In PlaceMarker\n")
		AutoLock lock(m_critSec);

		CHECK(ValidateOperation(OpPlaceMarker))
		Marker* marker = new Marker(eMarkerType, pvarMarkerValue, pvarContextValue);
		SampleData sampleData(ST_MARKER);
		sampleData.asMarker = std::shared_ptr<Marker>(marker);
		m_queueData.push(sampleData);

		// Unless we are paused, start an async operation to dispatch the next sample/marker.
		if(m_state != State_Paused)
			QueueAsyncOperation(OpPlaceMarker); // Increments ref count on pOp.

		return S_OK;
	}

	HRESULT HDStreamSink::Flush()
	{
		TRACE(L"In Flush\n")
		return S_OK;
	}

	HRESULT HDStreamSink::Shutdown()
	{
		TRACE(L"In Shutdown\n")

		return S_OK;
	}

	HRESULT HDStreamSink::IsMediaTypeSupported(IMFMediaType *pMediaType, IMFMediaType **ppMediaType)
	{
		TRACE(L"In IsMediaTypeSupported\n")
		if(pMediaType == nullptr)
			return E_INVALIDARG;

		AutoLock lock(m_critSec);

		GUID majorType = GUID_NULL;
		CHECK(pMediaType->GetGUID(MF_MT_MAJOR_TYPE, &majorType))
		if(majorType != MFMediaType_Video)
			return MF_E_INVALIDTYPE;

		GUID guiNewSubtype;
		CHECK(pMediaType->GetGUID(MF_MT_SUBTYPE, &guiNewSubtype))

		if (guiNewSubtype != MFVideoFormat_RGB24 && guiNewSubtype != MFVideoFormat_ARGB32 && guiNewSubtype != MFVideoFormat_L8 && guiNewSubtype != MFVideoFormat_L16 && guiNewSubtype != MFVideoFormat_D16)
			return MF_E_INVALIDTYPE;

		// We don't return any "close match" types.
		if(ppMediaType)
			*ppMediaType = nullptr;

		return S_OK;
	}

	HRESULT HDStreamSink::GetMediaTypeCount(DWORD *pdwTypeCount)
	{
		TRACE(L"In GetMediaTypeCount\n")
		if (pdwTypeCount == nullptr)
			return E_INVALIDARG;
		*pdwTypeCount = 1;

		return S_OK;
	}

	HRESULT HDStreamSink::GetMediaTypeByIndex(DWORD dwIndex, IMFMediaType **ppType)
	{
		TRACE(L"In GetMediaTypeByIndex\n")
		if (ppType == nullptr)
		{
			return E_INVALIDARG;
		}

		AutoLock lock(m_critSec);

		if(dwIndex > 0)
			return MF_E_NO_MORE_TYPES;
		else
		{
			*ppType = m_spCurrentType.Get();
			if(*ppType != nullptr)
				(*ppType)->AddRef();
		}
		return S_OK;
	}

	HRESULT HDStreamSink::SetCurrentMediaType(IMFMediaType *pMediaType)
	{
		TRACE(L"In SetCurrentMediaType\n")
		AutoLock lock(m_critSec);

		// We don't allow format changes after streaming starts.
		CHECK(ValidateOperation(OpSetMediaType))

		// We set media type already
		if (m_state >= State_Ready)
			CHECK(IsMediaTypeSupported(pMediaType, nullptr));

		CHECK(MFCreateMediaType(m_spCurrentType.ReleaseAndGetAddressOf()))
		CHECK(pMediaType->CopyAllItems(m_spCurrentType.Get()))

		if(m_state < State_Ready)
			m_state = State_Ready;

			
		else if(m_state > State_Ready)
		{
			Microsoft::WRL::ComPtr<IMFMediaType> spType;
			CHECK(MFCreateMediaType(&spType));
			CHECK(pMediaType->CopyAllItems(spType.Get()));
			ProcessFormatChange(spType.Get());
		}
		QueueEvent(MEStreamSinkFormatChanged, GUID_NULL, S_OK, nullptr);

		UINT32 width, height;
		if (FAILED(MFGetAttributeSize(m_spCurrentType.Get(), MF_MT_FRAME_SIZE, &width, &height)))
			return S_OK;
		m_streamWidth  = width;
		m_streamHeight = height;

		UINT32 framerateNum, framerateDenum;
		if (FAILED(MFGetAttributeRatio(m_spCurrentType.Get(), MF_MT_FRAME_RATE, &framerateNum, &framerateDenum)))
			return S_OK;

		GUID guiNewSubtype;
		CHECK(pMediaType->GetGUID(MF_MT_SUBTYPE, &guiNewSubtype))

		//Delete the old hand detector
		if (m_handDetector != NULL)
		{
			delete m_handDetector;
			m_handDetector = NULL;
		}

		//Create the hand detector
		//We detect between 25 and 80 centimeters, with a blob of minimum size = 900 pixels
		//Maximum wrist length : 100 pixels
		if (guiNewSubtype == MFVideoFormat_L8)
			m_handDetector = new HandDetection<D8Func>(width, height, 300, 800, 25, 500, 100);
		else if (guiNewSubtype == MFVideoFormat_D16 || guiNewSubtype == MFVideoFormat_L16)
			m_handDetector = new HandDetection<D16Func>(width, height, 300, 800, 25, 500, 100);
		
		m_mediaSubtype = guiNewSubtype;
		TRACE(L"Media type setted. Width: %d, Height: %d, framerate: %f, format: %ws\n", width, height, (float)framerateNum/framerateDenum, (guiNewSubtype == MFVideoFormat_D16 ? L"D16" : (guiNewSubtype == MFVideoFormat_RGB24 ? L"RGB24" : (guiNewSubtype == MFVideoFormat_L8 ? L"L8" : L"ARGB32"))))

		return S_OK;
	}

	HRESULT HDStreamSink::GetCurrentMediaType(IMFMediaType **ppMediaType)
	{
		TRACE(L"In GetCurrentMediaType")

		AutoLock lock(m_critSec);

		*ppMediaType = m_spCurrentType.Get();
		if(*ppMediaType)
			(*ppMediaType)->AddRef();
		return S_OK;
	}

	HRESULT HDStreamSink::GetMajorType(GUID *pguidMajorType)
	{
		TRACE(L"In GetMajorType")

		AutoLock lock(m_critSec);

		*pguidMajorType = MFMediaType_Video;
		return S_OK;
	}

	HRESULT HDStreamSink::Start(MFTIME start)
	{
		TRACE(L"In Start\n")

		AutoLock lock(m_critSec);

		CHECK(ValidateOperation(OpStart))

		m_state = State_Started;
		CHECK(QueueAsyncOperation(OpStart))

		return S_OK;
	}

	HRESULT HDStreamSink::Restart()
	{
		TRACE(L"In Restart\n")

		AutoLock lock(m_critSec);

		CHECK(ValidateOperation(OpRestart))

		m_state = State_Started;
		CHECK(QueueAsyncOperation(OpRestart))

		return S_OK;
	}

	HRESULT HDStreamSink::Stop()
	{
		TRACE(L"In Stop\n")

		AutoLock lock(m_critSec);

		CHECK(ValidateOperation(OpStop))

		m_state = State_Stopped;
		CHECK(QueueAsyncOperation(OpStop))

		return S_OK;
	}

	HRESULT HDStreamSink::Pause()
	{
		TRACE(L"In Pause\n")

		AutoLock lock(m_critSec);

		CHECK(ValidateOperation(OpPause))

		m_state = State_Paused;
		CHECK(QueueAsyncOperation(OpPause))

		return S_OK;
	}
	
	void HDStreamSink::ProcessFormatChange(IMFMediaType *pMediaType)
	{
		TRACE(L"In process format change\n")

		// Add the media type to the sample queue.
		SampleData sampleData(ST_MEDIATYPE);
		sampleData.asCOM = pMediaType;
		pMediaType->AddRef();
		m_queueData.push(sampleData);

		// Unless we are paused, start an async operation to dispatch the next sample.
		// Queue the operation.
		QueueAsyncOperation(OpSetMediaType);
	}


	// Puts an async operation on the work queue.
	HRESULT HDStreamSink::QueueAsyncOperation(StreamOperation op)
	{
		TRACE(L"In Queue Async Operation\n")
		AutoLock _lock(m_critSec);

		Microsoft::WRL::ComPtr<AsyncOperation> spOp;
		spOp.Attach(new AsyncOperation(op)); // Created with ref count = 1

		CHECK(MFPutWorkItem2(m_workQueueId, 0, &m_workQueueCB, spOp.Get()))
		return S_OK;
	}

	BOOL HDStreamSink::ValidateOperation(StreamOperation op) const
	{
		return validStateMatrix[m_state][op];
	}

	HRESULT HDStreamSink::OnDispatchWorkItem(IMFAsyncResult *pAsyncResult)
	{
		TRACE(L"New work Item received\n")

		Microsoft::WRL::ComPtr<IUnknown> spState;
		CHECK(pAsyncResult->GetState(&spState))

		// The state object is a AsyncOperation object.
		AsyncOperation *asyncOP = static_cast<AsyncOperation *>(spState.Get());
		StreamOperation op = asyncOP->op;

		switch(op)
		{
			case OpStart:
			case OpRestart:
			{
				TRACE(L"Sending Operation Start\n");
				AutoLock _lock(m_critSec);
				CHECK(QueueEvent(MEStreamSinkStarted, GUID_NULL, S_OK, nullptr))
				CHECK(QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, nullptr))
				TRACE(L"End Operation Start\n")
				break;
			}
			case OpPause:
			{
				AutoLock _lock(m_critSec);
				TRACE(L"Sending Operation Paused\n");
				CHECK(QueueEvent(MEStreamSinkPaused, GUID_NULL, S_OK, nullptr))
				break;
			}
			case OpStop:
			{
				AutoLock _lock(m_critSec);
				TRACE(L"Sending Operation Stopped\n");
				CHECK(QueueEvent(MEStreamSinkStopped, GUID_NULL, S_OK, nullptr))
				TRACE(L"End Operation Start\n")
				break;
			}
			case OpProcessSample:
			{
				TRACE(L"Processing sample in work item thread\n")

				//Fetch sample data
				IMFSample*  sample = NULL;
				{
					AutoLock _lock(m_critSec);
					SampleData& sd = m_queueData.front();
					sample = static_cast<IMFSample*>(sd.asCOM.Get());
					m_queueData.pop();
				}

				MFPinholeCameraIntrinsics cameraIntrinsics;
				sample->GetBlob(MFSampleExtension_PinholeCameraIntrinsics, (UINT8*)(&cameraIntrinsics), 1, NULL);

				IMFMediaBuffer* sampleBuffer;
				unsigned long bufferCount;
				sample->GetBufferCount(&bufferCount);
				sample->ConvertToContiguousBuffer(&sampleBuffer);
				BYTE* rawBuffer;
				DWORD bufferCurrentLength;
				sampleBuffer->Lock(&rawBuffer, NULL, &bufferCurrentLength);
								
				//Update the hand position
				CHECK(UpdateHandDetection(sample, rawBuffer, bufferCurrentLength))

				//Release the frame
				sampleBuffer->Unlock();
				sampleBuffer->Release();
				sample->Release();
				break;
			}

			case OpPlaceMarker:
			{
				AutoLock _lock(m_critSec);
				QueueEvent(MEStreamSinkMarker, GUID_NULL, S_OK, nullptr);
				m_queueData.pop();
				break;
			}

			case OpSetMediaType:
			{
				AutoLock _lock(m_critSec);
				m_queueData.pop();
				break;
			}
			default:
				break;
		}
		TRACE(L"End onWorkItem received\n")
		return S_OK;
	}

	HRESULT HDStreamSink::UpdateHandDetection(IMFSample* sample, BYTE* rawBuffer, unsigned long bufferCurrentLength)
	{
		if (m_handDetector != NULL)
		{
			//Update the status
			m_handDetector->updateDetection(rawBuffer);

			//Call the callback interface
			if (m_clbk != nullptr)
			{
				Platform::Collections::Vector<HandDetector_Native::Hand^>^ hands = ref new Platform::Collections::Vector<HandDetector_Native::Hand^>();

				//Determine what depth function to use
				DepthFunc depthFunc = NULL;
				if (m_mediaSubtype == MFVideoFormat_D16 || m_mediaSubtype == MFVideoFormat_L16)
					depthFunc = &D16Func::depthAt;
				else if (m_mediaSubtype == MFVideoFormat_L8)
					depthFunc = &D8Func::depthAt;

				//Get the camera parameters
				HandDetector_Native::CameraParameter camera;

				//First the spatial coordinate system
				Microsoft::WRL::ComPtr<IUnknown> spUnknown;
				Microsoft::WRL::ComPtr<IInspectable> spSpatialCoordinateSystem = NULL;
				if(SUCCEEDED(sample->GetUnknown(MFSampleExtension_Spatial_CameraCoordinateSystem, IID_PPV_ARGS(&spUnknown))))
					spUnknown.As(&spSpatialCoordinateSystem);

				
				memset(&camera.CameraViewTransform, 0, sizeof(camera.CameraViewTransform));
				memset(&camera.CameraProjectionTransform, 0, sizeof(camera.CameraProjectionTransform));
				memset(&camera.CameraIntrinsics, 0, sizeof(camera.CameraIntrinsics));

				//Then camera parameters
				UINT32 blobSize;
				if(FAILED(sample->GetBlob(MFSampleExtension_Spatial_CameraViewTransform, (UINT8*)(&camera.CameraViewTransform), sizeof(camera.CameraViewTransform), &blobSize)))
					TRACE("Failed to get camera view transform\n")
				if(FAILED(sample->GetBlob(MFSampleExtension_Spatial_CameraProjectionTransform, (UINT8*)(&camera.CameraProjectionTransform), sizeof(camera.CameraProjectionTransform), &blobSize)))
					TRACE("Failed to get camera projection transform\n")
				if(FAILED(sample->GetBlob(MFSampleExtension_PinholeCameraIntrinsics, (UINT8*)&camera.CameraIntrinsics, sizeof(camera.CameraIntrinsics), &blobSize)))
					TRACE("Failed to get pinhole camera intrinsics\n")

				SensorStreaming::ICameraIntrinsics* sensorStreamingCameraIntrinsics = NULL;
				if(SUCCEEDED(sample->GetUnknown(SensorStreaming::MFSampleExtension_SensorStreaming_CameraIntrinsics, IID_PPV_ARGS(&spUnknown))))
					sensorStreamingCameraIntrinsics = (SensorStreaming::ICameraIntrinsics*)spUnknown.Get();

				//If found
				if(depthFunc)
				{
					//Create the WinRT proxy object
					if (sensorStreamingCameraIntrinsics == NULL)
					{
						for (const auto& h : m_handDetector->getHands())
						{
							HandDetector_Native::Hand^ hand = ref new HandDetector_Native::Hand();
							hand->InPixels = true;

							//Palm position
							hand->PalmX = h.palmX;
							hand->PalmY = h.palmY;
							hand->PalmZ = depthFunc(h.palmX, h.palmY, m_streamWidth, rawBuffer);

							//Fingers
							for (const auto& f : h.fingers)
							{
								HandDetector_Native::Finger finger;
								finger.TipX = f.tipX;
								finger.TipY = f.tipY;
								finger.TipZ = depthFunc(f.tipX, f.tipY, m_streamWidth, rawBuffer);
								hand->Fingers->Append(finger);
							}

							//Wrist
							hand->WristX = h.wristPosX;
							hand->WristY = h.wristPosY;
							hand->WristZ = depthFunc((uint16_t)hand->WristX, (uint16_t)hand->WristY, m_streamWidth, rawBuffer);

							hands->Append(hand);
						}
					}
					else
					{
						for (const auto& h : m_handDetector->getHands())
						{
							HandDetector_Native::Hand^ hand = ref new HandDetector_Native::Hand();

							//Palm Position
							float outXY[2];
							float inXY[2] = { (float)h.palmX, (float)h.palmY};
							float depth = -depthFunc(h.palmX, h.palmY, m_streamWidth, rawBuffer)/1000.0f;
							
							sensorStreamingCameraIntrinsics->MapImagePointToCameraUnitPlane(inXY, outXY);
							float z = depth / sqrt(outXY[0] * outXY[0] + outXY[1] * outXY[1] + 1);

							hand->PalmX = z*outXY[0];
							hand->PalmY = z*outXY[1];
							hand->PalmZ = z;

							//Fingers
							for (const auto& f : h.fingers)
							{
								HandDetector_Native::Finger finger;

								inXY[0] = f.tipX;
								inXY[1] = f.tipY;
								depth = -depthFunc(f.tipX, f.tipY, m_streamWidth, rawBuffer)/1000.0f;
								sensorStreamingCameraIntrinsics->MapImagePointToCameraUnitPlane(inXY, outXY);
								z = depth / sqrt(outXY[0] * outXY[0] + outXY[1] * outXY[1] + 1);

								finger.TipX = z*outXY[0];
								finger.TipY = z*outXY[1];
								finger.TipZ = z;
								hand->Fingers->Append(finger);
							}

							//Wrist positions
							inXY[0] = h.wristPosX;
							inXY[1] = h.wristPosY;
							depth = -depthFunc(h.wristPosX, h.wristPosY, m_streamWidth, rawBuffer) / 1000.0f;
							sensorStreamingCameraIntrinsics->MapImagePointToCameraUnitPlane(inXY, outXY);

							z = depth / sqrt(outXY[0] * outXY[0] + outXY[1] * outXY[1] + 1);
							hand->WristX = z * outXY[0];
							hand->WristY = z * outXY[1];
							hand->WristZ = z;

							//Copy the ROIs information
							hand->BlobROIMinX = h.blobMinROI[0];
							hand->BlobROIMinY = h.blobMinROI[1];
							hand->BlobROIMaxX = h.blobMaxROI[0];
							hand->BlobROIMaxY = h.blobMaxROI[1];

							hand->WristROIMinX = h.wristMinROI[0];
							hand->WristROIMinY = h.wristMinROI[1];
							hand->WristROIMaxX = h.wristMaxROI[0];
							hand->WristROIMaxY = h.wristMaxROI[1];

							hands->Append(hand);
						}
					}
				}

				//Call the interface
				m_clbk->OnHandUpdate(camera, safe_cast<Windows::Perception::Spatial::SpatialCoordinateSystem^>(reinterpret_cast<Platform::Object^>(spSpatialCoordinateSystem.Get())), hands);
			}
		}
		return S_OK;
	}

	HRESULT HDStreamSink::WriteSampleToFile(BYTE* rawBuffer, unsigned long bufferCurrentLength)
	{
		uint32_t width;
		uint32_t height;

		Platform::String^ strPath = Windows::Storage::ApplicationData::Current->LocalFolder->Path;
		const wchar_t* path = strPath->Data();
		wchar_t fileName[2048];
		swprintf_s(fileName, 2048, L"%s/frame%d.ppm", path, m_frameID++);
		FILE* f;
		_wfopen_s(&f, fileName, L"w");
		TRACE(L"Writing to %s\n", fileName)

		char header[256];
		{
			AutoLock _lock(m_critSec);
			CHECK(MFGetAttributeSize(m_spCurrentType.Get(), MF_MT_FRAME_SIZE, &width, &height))
		}
		sprintf_s(header, 256, "P2\n%d %d\n%d", width, height, (1 << 16) - 1);
		fwrite(header, sizeof(char), strlen(header), f);

		std::string txt;
		for (DWORD i = 0; i < bufferCurrentLength / 2; i++)
		{
			if (i % 70 == 0)
				txt.push_back('\n');

			char intStr[16];
			sprintf_s(intStr, 16, "%d ", (int)((int)rawBuffer[2 * i] + (int)(rawBuffer[2 * i + 1] << 8)));
			txt += intStr;
		}
		fwrite(txt.c_str(), sizeof(char), txt.length(), f);
		TRACE(L"End writting")
		fclose(f);
		return S_OK;
	}
}