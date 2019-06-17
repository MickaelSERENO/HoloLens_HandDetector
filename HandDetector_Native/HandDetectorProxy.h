#pragma once

#include <HandDetector.h>
#include <collection.h>
#include <cstdint>
#include <functional>

namespace HandDetector_Native
{
	/** Structure representing a Finger*/
	public value struct Finger
	{
		float TipX; /*!< The Tip X position in the camera coordinate system (not the image)*/
		float TipY; /*!< The Tip Y position in the camera coordinate system (not the image)*/
		float TipZ; /*!< The Tip Z position in the camera coordinate system (not the image)*/
	};

	/** Functor permitting to compare two fingers*/
	struct FingerEqual
	{
		bool operator()(const Finger& left, const Finger& right) const
		{
			return left.TipX == right.TipX &&
				   left.TipY == right.TipY &&
				   left.TipZ == right.TipZ;
		};
	};

	/** Class representing a Hand*/
	public ref class Hand sealed
	{
		public:
			/** Constructor. Initial every value to 0*/
			Hand();

			/** Destructor*/
			virtual ~Hand();

			/*!< The Finger collection*/
			property Windows::Foundation::Collections::IVector<Finger>^ Fingers {Windows::Foundation::Collections::IVector<Finger>^ get() { return m_fingers; }}

			/*!< The Hand X position in the camera coordinate system (not the image)*/
			property float PalmX;

			/*!< The Hand X position in the camera coordinate system (not the image)*/
			property float PalmY;

			/*!< The Hand Z position in the camera coordinate system (not the image)*/
			property float PalmZ;

			/*!< In the case of not being able to fetch intrinsics camera parameters, the coordinate are given in pixels. This property will then be set to true*/
			property bool InPixels;

			/*!< The min X ROI of the detected blob where the hand belongs to in the image (pixels) space*/
			property int BlobROIMinX;

			/*!< The min Y ROI of the detected blob where the hand belongs to in the image (pixels) space*/
			property int BlobROIMinY;

			/*!< The max X ROI of the detected blob where the hand belongs to in the image (pixels) space*/
			property int BlobROIMaxX;

			/*!< The max Y ROI of the detected blob where the hand belongs to in the image (pixels) space*/
			property int BlobROIMaxY;

			/*!< The min X ROI of the detected hand (fingers to wrist) in the image (pixels) space*/
			property int WristROIMinX;

			/*!< The min Y ROI of the detected hand (fingers to wrist) in the image (pixels) space*/
			property int WristROIMinY;

			/*!< The max X ROI of the detected hand (fingers to wrist) in the image (pixels) space*/
			property int WristROIMaxX;

			/*!< The max Y ROI of the detected hand (fingers to wrist) in the image (pixels) space*/
			property int WristROIMaxY;

			/*!< The wrist X position*/
			property float WristX;

			/*The wrist Y position*/
			property float WristY;

			/*The wrist Z position*/
			property float WristZ;
		private:
			Windows::Foundation::Collections::IVector<Finger>^ m_fingers; /*!< The fingers array*/
	};
}