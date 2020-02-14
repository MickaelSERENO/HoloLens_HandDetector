#pragma once

#include <wrl.h>
#include <memory>

#include "Marker.h"

namespace Sereno
{
	/** Type of a sample received*/
	enum SampleType
	{
		ST_NO_TYPE, /** No sample*/
		ST_MARKER,  /** Marker sample*/
		ST_SAMPLE,  /** Sample (image) sample*/
		ST_MEDIATYPE /** Changement in the media type*/
	};

	/** SampleData class. Permits to store the SampleType and its value in a union structure*/
	struct SampleData
	{
		SampleData(SampleType t) : type(ST_NO_TYPE)
		{
			setType(t);
		}

		~SampleData()
		{
			setType(ST_NO_TYPE);
		}

		SampleData(const SampleData& cpy)
		{
			type = ST_NO_TYPE;
			*this = cpy;
		}

		SampleData& operator=(const SampleData& cpy)
		{
			setType(ST_NO_TYPE);
			switch(cpy.type)
			{
				case ST_MARKER:
					new (&asMarker) std::shared_ptr<Marker>(cpy.asMarker);
					break;
				case ST_SAMPLE:
				case ST_MEDIATYPE:
					new (std::addressof(asCOM)) Microsoft::WRL::ComPtr<IUnknown>(cpy.asCOM);
					break;
			}
			type = cpy.type;
			return *this;
		}

		SampleType type;

		void setType(SampleType t)
		{
			switch (type)
			{
				case ST_MARKER:
					asMarker.~shared_ptr();
					break;
				case ST_SAMPLE:
				case ST_MEDIATYPE:
					asCOM.~ComPtr();
					break;
			}

			type = t;

			switch (type)
			{
				case ST_MARKER:
					new (&asMarker) std::shared_ptr<Marker>;
					break;
				case ST_SAMPLE:
				case ST_MEDIATYPE:
					new (std::addressof(asCOM)) Microsoft::WRL::ComPtr<IUnknown>;
					break;
			}
		}

		union
		{
			std::shared_ptr<Marker>          asMarker;
			Microsoft::WRL::ComPtr<IUnknown> asCOM;
		};
	};
}