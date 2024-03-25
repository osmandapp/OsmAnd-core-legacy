#ifndef _OSMAND_ELAPSED_TIMER_H_
#define _OSMAND_ELAPSED_TIMER_H_

#include <chrono>
using namespace std::chrono;

namespace OsmAnd {
class ElapsedTimer {
   private:
	bool isEnabled;
	bool isRunning;
	high_resolution_clock::duration elapsed;
	high_resolution_clock::time_point startPoint;

   public:
	ElapsedTimer();

	void Enable();
	void Disable();

	// starts new after reset or resumes previous run
	void Restart();
	void Start();
	void Pause();
	void Reset();

	const high_resolution_clock::duration& GetElapsed();

	uint64_t GetElapsedNanos();
	uint64_t GetElapsedMicros();
	uint64_t GetElapsedMs();
};
}  // namespace OsmAnd

#endif
