#pragma once

#include <Windows.h>

namespace Sereno
{
	/** Class permitting to handle thread safety*/
	class CritSec
	{
		public:
			CRITICAL_SECTION m_criticalSection; /** The "mutex" object*/

			CritSec()
			{
				InitializeCriticalSectionEx(&m_criticalSection, 100, 0);
			}

			~CritSec()
			{
				DeleteCriticalSection(&m_criticalSection);
			}

			/** Lock this object*/
			_Acquires_lock_(m_criticalSection)
			void Lock()
			{
				EnterCriticalSection(&m_criticalSection);
			}

			/** Unlock this object*/
			_Releases_lock_(m_criticalSection)
			void Unlock()
			{
				LeaveCriticalSection(&m_criticalSection);
			}
	};

	/** Object permitting to automatically lock and unlock a CritSec object. Lock the object while the destructor is not called*/
	class AutoLock
	{
		public:
			_Acquires_lock_(m_critiSec)
			AutoLock(CritSec& crit)
			{
				m_critiSec = &crit;
				m_critiSec->Lock();
			}

			_Releases_lock_(m_critiSec)
			~AutoLock()
			{
				m_critiSec->Unlock();
			}

		private:
			CritSec *m_critiSec; /** The critical section object*/
	};
}