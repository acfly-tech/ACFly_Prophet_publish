#include "FreeRTOS.h"
#include "task.h"
#include "TimeBase.hpp"
#include "Commulink.hpp"   // MAVLink send path: get_CommuPort, get_CommulinkSysId/CompId, mavlink.h
#include <string.h>

// ---------------------------------------------------------------------------
// Overall CPU load (idle-time based, updated every 0.5 s)
// ---------------------------------------------------------------------------
static float CPULoad = 0;
static TIME IdleTime(false);
static TIME IdleStartTime(false);
// Tracks whether the idle task is currently executing.
// Both SysTick and PendSV run at the lowest priority (255) on Cortex-M and
// cannot preempt each other, so this flag needs no additional protection.
static bool inIdle = false;

extern "C" void EnterIdleTask()
{
	inIdle = true;
	IdleStartTime = TIME::now();
}
extern "C" void ExitIdleTask()
{
	inIdle = false;
	TIME passTime = TIME::now();
	passTime -= IdleStartTime;
	IdleTime += passTime;
}
extern "C" void vApplicationTickHook()
{
	static uint16_t ticks = 0;
	if( ++ticks >= configTICK_RATE_HZ/2 )
	{	// Refresh CPU load every 0.5 seconds.
		ticks = 0;

		// New FreeRTOS (DEVELOPMENT BRANCH) calls vApplicationTickHook even
		// while the scheduler is suspended (else-branch of xTaskIncrementTick).
		// During suspension no context switches occur, so ExitIdleTask is never
		// called and IdleStartTime grows stale.  If the window boundary falls
		// while the idle task is running we must:
		//   1. Credit the idle time accumulated so far into this window.
		//   2. Move IdleStartTime to NOW so the next ExitIdleTask only counts
		//      time from this boundary onward, not from before the reset.
		if (inIdle)
		{
			TIME now = TIME::now();
			TIME passTime = now;
			passTime -= IdleStartTime;
			IdleTime += passTime;
			IdleStartTime = now;
		}

		CPULoad = 100 - IdleTime.get_pass_time_fromStartUp() / 0.5f * 100;
		IdleTime.set_invalid();
	}
}

// Total CPU load derived from run-time stats (100% minus idle task share).
// Updated every 0.5 s alongside taskCPULoads[].
volatile float   taskCPULoadTotal = 0.0f;


extern "C" float getCPULoad()
{
	return CPULoad;
	//return taskCPULoadTotal;
}

// ---------------------------------------------------------------------------
// Run-time stats counter: TIM5 at 10 MHz, required by
// configGENERATE_RUN_TIME_STATS / portGET_RUN_TIME_COUNTER_VALUE().
// ---------------------------------------------------------------------------
extern "C" uint32_t getRunTimeCounterValue(void)
{
	return TIM5->CNT;
}

// ---------------------------------------------------------------------------
// Per-task CPU load — refreshed every CPU_STATS_WINDOW_MS.
// Watch taskCPULoads[0..taskCPULoadCount-1] in the debugger.
//
// IMPORTANT: the array order MUST stay stable.  The element index doubles as
// the THREAD_INFO.index reported over MAVLink, so we never re-sort the array
// by CPU usage (see refresh_thread_stats / send_one_thread_info).
// ---------------------------------------------------------------------------
#define CPU_STATS_MAX_TASKS  32

// Base tick of cpu_stats_task.
#define CPU_STATS_TICK_MS    1000
// Per-task CPU load window.
//
// uxTaskGetSystemState() internally calls vTaskSuspendAll()/xTaskResumeAll(),
// which FREEZES the scheduler (no task context switches) for the whole scan.
// Interrupts still run, but any task (attitude/position control loops that
// rely on vTaskDelayUntil scheduling) due to run during that window is delayed
// until xTaskResumeAll() returns — causing timing jitter.  On a flight
// controller this is a real-time hazard, so we deliberately keep the refresh
// rate LOW: per-thread stats are for GCS display/debugging only and 2 Hz is
// already overkill.  Raising this window from 500 ms to 2 s cuts how often the
// scheduler is frozen to one quarter, at no practical cost in observability.
#define CPU_STATS_WINDOW_MS  2000
// Round-robin THREAD_INFO MAVLink send period: one thread is reported per
// tick, cycling through all threads.  Keeping a low rate (rather than dumping
// all ~20 threads at once) avoids MAVLink packet bursts that can cause RX
// drops on lower-bandwidth UART links.
#define THREAD_INFO_SEND_MS  400
// If a thread's CPU occupancy changes by more than this many percentage
// points versus the value carried in the last THREAD_INFO we sent for it,
// it is reported immediately (outside the round-robin cadence) so the GCS
// sees sharp load spikes without waiting for the index pointer to reach it.
#define CPU_LOAD_DELTA_TRIGGER  3.0f

#if ( configUSE_TRACE_FACILITY == 1 )

struct TaskCPULoad
{
	char     name[configMAX_TASK_NAME_LEN]; // task name
	float    cpuPercent;                    // share of CPU over the last CPU_STATS_WINDOW_MS
	float    prevCpuSentPercent;            // cpuPercent value carried by the last THREAD_INFO we sent for this thread (-1 = never sent)
	uint16_t stackFreeWords;                // minimum free stack ever seen (words)
	uint16_t stackSizeWords;                // estimated total stack size (words)
	uint8_t  priority;                      // RTOS native priority
	uint8_t  state;                         // FreeRTOS eTaskState (mirrors THREAD_STATE)
};

volatile uint8_t taskCPULoadCount = 0;
TaskCPULoad      taskCPULoads[CPU_STATS_MAX_TASKS];
// When actual task count exceeds CPU_STATS_MAX_TASKS this is set to the
// number of tasks that were not shown (clipped). 0 means no clipping.
volatile uint8_t taskCPUStatsOverflow = 0;

// Forward declaration: send_thread_info is defined after cpu_stats_task but
// used inside it (delta-trigger + round-robin paths share the same helper).
static void send_thread_info(uint8_t idx);

static void cpu_stats_task(void *pvParam)
{
	(void)pvParam;
	// prevCounters / prevHandles track the last run-time counter value per
	// task handle so we can compute per-window deltas.
	static TaskStatus_t  states[CPU_STATS_MAX_TASKS];
	static uint32_t      prevCounters[CPU_STATS_MAX_TASKS];
	static TaskHandle_t  prevHandles[CPU_STATS_MAX_TASKS];
	uint8_t prevCount = 0;

	uint32_t windowTick = pdMS_TO_TICKS(CPU_STATS_WINDOW_MS) / pdMS_TO_TICKS(CPU_STATS_TICK_MS);
	if (windowTick == 0)
		windowTick = 1;
	uint8_t tickCount = 0;

	TickType_t xLastWake = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(CPU_STATS_TICK_MS));

		bool refreshWindow = (++tickCount >= windowTick);
		if (refreshWindow)
			tickCount = 0;

		if (refreshWindow)
		{ // Refresh per-task CPU load every CPU_STATS_WINDOW_MS.
			UBaseType_t n = uxTaskGetSystemState(states, CPU_STATS_MAX_TASKS, NULL);
			if (n == 0) continue;

			// Sum all per-task deltas to get the total run-time for this window.
			uint32_t deltaTotal = 0;
			for (UBaseType_t i = 0; i < n; i++)
			{
				uint32_t prev = 0;
				for (uint8_t j = 0; j < prevCount; j++)
				{
					if (prevHandles[j] == states[i].xHandle) { prev = prevCounters[j]; break; }
				}
				deltaTotal += states[i].ulRunTimeCounter - prev;
			}
			if (deltaTotal == 0) deltaTotal = 1;

			uint8_t cnt = (n < CPU_STATS_MAX_TASKS) ? (uint8_t)n : CPU_STATS_MAX_TASKS;
			TaskCPULoad buf[CPU_STATS_MAX_TASKS];
			float idlePercent = 0.0f;

			for (UBaseType_t i = 0; i < n; i++)
			{
				uint32_t prev = 0;
				for (uint8_t j = 0; j < prevCount; j++)
				{
					if (prevHandles[j] == states[i].xHandle) { prev = prevCounters[j]; break; }
				}
				uint32_t delta = states[i].ulRunTimeCounter - prev;
				float pct = (float)delta * 100.0f / (float)deltaTotal;

				// Track idle task share to compute total CPU load regardless of
				// whether the idle task makes it into the display array.
				if (states[i].uxCurrentPriority == tskIDLE_PRIORITY && pct > idlePercent)
					idlePercent = pct;

				if (i >= CPU_STATS_MAX_TASKS) continue; // clipped — counted but not displayed

				buf[i].cpuPercent     = pct;
				buf[i].stackFreeWords = states[i].usStackHighWaterMark;
				buf[i].stackSizeWords = states[i].usStackHighWaterMark; // updated below
				buf[i].priority       = (uint8_t)states[i].uxCurrentPriority;
				buf[i].state          = (uint8_t)states[i].eCurrentState;
				const char *src = states[i].pcTaskName;
				char       *dst = buf[i].name;
				int k = 0;
				while (k < configMAX_TASK_NAME_LEN - 1 && src[k]) { dst[k] = src[k]; k++; }
				dst[k] = '\0';
			}

			// Compute the real total stack size in words from the stack base
			// and end pointers.  Requires configRECORD_STACK_HIGH_ADDRESS == 1
			// (set in FreeRTOSConfig.h) so TaskStatus_t carries pxEndOfStack;
			// otherwise we fall back to 2*highWaterMark as a lower-bound guess.
			for (uint8_t i = 0; i < cnt; i++)
			{
#if ( configRECORD_STACK_HIGH_ADDRESS == 1 )
				if (states[i].pxEndOfStack >= states[i].pxStackBase)
				{
					buf[i].stackSizeWords = (uint16_t)(states[i].pxEndOfStack - states[i].pxStackBase);
				}
				else
#endif
				{
					buf[i].stackSizeWords = buf[i].stackFreeWords * 2;
				}
			}

			// Save current counters for the next delta computation.
			prevCount = cnt;
			for (uint8_t i = 0; i < cnt; i++)
			{
				prevHandles[i]  = states[i].xHandle;
				prevCounters[i] = states[i].ulRunTimeCounter;
			}

			// Merge buf into taskCPULoads keeping a STABLE order keyed by task
			// name.  Existing rows are updated in place, brand-new tasks are
			// appended at the end, and vanished tasks are dropped by shifting
			// the tail down.  This keeps element indices (== THREAD_INFO.index)
			// from jumping around between sends.
			bool bufUsed[CPU_STATS_MAX_TASKS] = {false};

			// First pass: map each existing display slot to a fresh buf entry.
			uint8_t writePos = 0;
			for (uint8_t i = 0; i < taskCPULoadCount; i++)
			{
				for (uint8_t k = 0; k < cnt; k++)
				{
					if (bufUsed[k]) continue;
					bool match = true;
					for (int c = 0; c < configMAX_TASK_NAME_LEN; c++)
					{
						if (taskCPULoads[i].name[c] != buf[k].name[c]) { match = false; break; }
						if (buf[k].name[c] == '\0') break;
					}
					if (match)
					{
						// Struct copy would clobber prevCpuSentPercent with
						// garbage from the uninitialized local buf[].  Copy
						// volatile measurement fields only, preserving the
						// per-thread send-state fields that live across windows.
						float savedPrev = taskCPULoads[writePos].prevCpuSentPercent;
						taskCPULoads[writePos]    = buf[k];
						taskCPULoads[writePos].prevCpuSentPercent = savedPrev;
						bufUsed[k]                = true;
						writePos++;
						break;
					}
				}
			}

			// Second pass: append newly-seen tasks.
			for (uint8_t k = 0; k < cnt && writePos < CPU_STATS_MAX_TASKS; k++)
			{
				if (!bufUsed[k])
				{
					taskCPULoads[writePos] = buf[k];
					// New task: force an initial report in the delta-trigger
					// pass below (prevCpuSentPercent == -1 => diff = 1000).
					taskCPULoads[writePos].prevCpuSentPercent = -1.0f;
					writePos++;
				}
			}

			taskCPULoadCount    = writePos;
			taskCPUStatsOverflow = (n > CPU_STATS_MAX_TASKS) ? (uint8_t)(n - CPU_STATS_MAX_TASKS) : 0;
			taskCPULoadTotal    = 100.0f - idlePercent;

			// Delta-trigger pass: any thread whose CPU load changed by more
			// than CPU_LOAD_DELTA_TRIGGER percentage points since its last
			// report is emitted immediately, overriding the round-robin cadence
			// so the GCS sees sharp load spikes quickly.  This fires only on a
			// refresh tick and never produces a burst (bounded by writePos).
			for (uint8_t i = 0; i < writePos; i++)
			{
				float prev = taskCPULoads[i].prevCpuSentPercent;
				float diff = (prev < 0.0f) ? 1000.0f : (taskCPULoads[i].cpuPercent - prev);
				if (diff < 0.0f) diff = -diff;
				if (diff >= CPU_LOAD_DELTA_TRIGGER)
					send_thread_info(i);
			}
		}

		// Round-robin THREAD_INFO report: emit exactly one thread per
		// THREAD_INFO_SEND_MS so the GCS never sees a burst.
		{
			static uint16_t sendAccumMs = 0;
			static uint8_t  sendIdx     = 0;
			sendAccumMs += CPU_STATS_TICK_MS;
			if (sendAccumMs >= THREAD_INFO_SEND_MS)
			{
				sendAccumMs = 0;

				uint8_t cnt = taskCPULoadCount;
				if (cnt > 0)
				{
					if (sendIdx >= cnt) sendIdx = 0;

					send_thread_info(sendIdx);

					++sendIdx;
					if (sendIdx >= cnt) sendIdx = 0;
				}
			}
		}
	}
}

// Pack and send one THREAD_INFO message for taskCPULoads[idx] on every
// connected MAVLink channel.  Updates prevCpuSentPercent so the delta
// trigger knows the baseline for the next comparison.
static void send_thread_info(uint8_t idx)
{
	if (idx >= taskCPULoadCount) return;
	const TaskCPULoad *t = &taskCPULoads[idx];

	mavlink_thread_info_t info;
	info.time_boot_ms    = (uint32_t)(TIME::get_System_Run_Time() * 1000.0);
	info.remaining_stack = t->stackFreeWords;
	info.stack_size      = t->stackSizeWords;
	info.index           = idx;
	info.num_threads     = taskCPULoadCount;
	// Thread CPU usage in centi-percent (0-10000). 10000 == 100.00%,
	// so two decimal places are preserved without a float field.
	float clamped = (t->cpuPercent < 0.0f) ? 0.0f : (t->cpuPercent > 100.0f ? 100.0f : t->cpuPercent);
	info.cpu_usage       = (uint16_t)(clamped * 100.0f + 0.5f);
	info.priority        = t->priority;
	info.state           = t->state;
	memset(info.name, 0, sizeof(info.name));
	strncpy(info.name, t->name, sizeof(info.name) - 1);

	for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
	{
		const Port *port = get_CommuPort(i);
		if (port && port->write != 0)
		{
			if (mavlink_lock_chan(i, 0.01))
			{
				mavlink_message_t msg_sd;
				mavlink_msg_thread_info_pack_chan(get_CommulinkSysId(),
				                                  get_CommulinkCompId(),
				                                  i, // chan
				                                  &msg_sd,
				                                  info.time_boot_ms,
				                                  info.remaining_stack,
				                                  info.stack_size,
				                                  info.cpu_usage,
				                                  info.index,
				                                  info.num_threads,
				                                  info.priority,
				                                  info.state,
				                                  info.name);
				mavlink_send_msg(port, &msg_sd, 0, 0.1);
				mavlink_unlock_chan(i);
			}
		}
	}

	// Remember what we sent so the delta trigger has a baseline.
	taskCPULoads[idx].prevCpuSentPercent = t->cpuPercent;
}

#endif

extern "C" void startCPUStatsTask(void)
{
	#if ( configUSE_TRACE_FACILITY == 1 )
	xTaskCreate(cpu_stats_task, "CPUStats", 1024, NULL, 1, NULL);
	#endif
}
