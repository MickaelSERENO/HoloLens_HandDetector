#include "HandDetectorProxy.h"

namespace HandDetector_Native
{
	Hand::Hand()
	{
		m_fingers = ref new Platform::Collections::Vector<Finger, FingerEqual>();
		InPixels = false;
	}

	Hand::~Hand() {}
}