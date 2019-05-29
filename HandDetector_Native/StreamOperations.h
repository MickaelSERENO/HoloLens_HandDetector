#pragma once

namespace Sereno
{
	// State enum: Defines the current state of the stream.
	enum State
	{
		State_TypeNotSet = 0,    // No media type is set
		State_Ready,             // Media type is set, Start has never been called.
		State_Started,
		State_Stopped,
		State_Paused,
		State_Count              // Number of states
	};

	// StreamOperation: Defines various operations that can be performed on the stream.
	enum StreamOperation
	{
		OpSetMediaType = 0,
		OpStart,
		OpRestart,
		OpPause,
		OpStop,
		OpProcessSample,
		OpPlaceMarker,
		Op_Count                // Number of operations
	};
}