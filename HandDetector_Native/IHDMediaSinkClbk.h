#pragma once

#include <collection.h>

#include "HandDetectorProxy.h"

namespace HandDetector_Native
{
	/** Corresponds to MF_FLOAT2*/
	public value struct FLOAT2
	{
		FLOAT x;
		FLOAT y;
	};

	/** Corresponds to MFCameraIntrinsic_PinholeCameraModel */
	public value struct CameraIntrinsic_PinholeCameraModel 
	{
		FLOAT2 FocalLength;
		FLOAT2 PrincipalPoint;
	};

	/** Corresponds to  MFCameraIntrinsic_DistortionModel */
	public value struct CameraIntrinsic_DistortionModel 
	{
		FLOAT Radial_k1;
		FLOAT Radial_k2;
		FLOAT Radial_k3;
		FLOAT Tangential_p1;
		FLOAT Tangential_p2;
	};

	/** Corresponds to  MFPinholeCameraIntrinsic_IntrinsicModel */
	public value struct PinholeCameraIntrinsic_IntrinsicModel
	{
		UINT32                             Width;
		UINT32                             Height;
		CameraIntrinsic_PinholeCameraModel CameraModel;
		CameraIntrinsic_DistortionModel    DistortionModel;
	};

	/** Corrsponds to MFCameraIntrinsics*/
	public value struct CameraIntrinsics
	{
		UINT32                                IntrinsicModelCount;
		PinholeCameraIntrinsic_IntrinsicModel IntrinsicModels;
	};

	/** The camera parameter for a given frame*/
	public value struct CameraParameter
	{
		/** The Camera View Transform matrix*/
		Windows::Foundation::Numerics::float4x4 CameraViewTransform;

		/** The Camera Projection Transform matrix*/
		Windows::Foundation::Numerics::float4x4 CameraProjectionTransform;

		/** The Camera Intrinsics Parameters*/
		CameraIntrinsics CameraIntrinsics;
	};

	/** Interface class handling Hand Detector Media Sink callback*/
	public interface class IHDMediaSinkClbk
	{
		/** Function called when the position of the hands have been updated
		 * \param hands the new hands detected*/
		void OnHandUpdate(CameraParameter cameraParam, Windows::Perception::Spatial::SpatialCoordinateSystem^ spatialCoordinateSystem, 
			              Windows::Foundation::Collections::IVector<HandDetector_Native::Hand^>^ hands);
	};
}