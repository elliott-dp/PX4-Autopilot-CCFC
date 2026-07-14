// Minimal stubs satisfying TECS.cpp link requirements in unit test builds.
// PX4_WARN calls in TECS.cpp are only reached on invalid dt; they are never
// triggered by the closed-loop tests which always pass dt = 0.02 s.

#include <cstdarg>
#include <cstdio>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/sem.h>

extern "C" __attribute__((visibility("default")))
void px4_log_modulename(int /*level*/, const char * /*module*/, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
}

// Stubs for work_queue internals pulled in transitively via px4_layer.
// The HRT work queue thread is never started in unit tests.
px4_task_t px4_task_spawn_cmd(const char * /*name*/, int /*scheduler*/, int /*priority*/,
			      int /*stack_size*/, px4_main_t /*entry*/, char *const /*argv*/[])
{
	return -1;
}

int px4_task_kill(px4_task_t /*id*/, int /*sig*/) { return 0; }

extern "C" px4_task_t px4_getpid() { return -1; }

// px4_sem stubs for the HRT work queue objects (hrt_thread.c) pulled in
// transitively via px4_layer's drv_hrt. The work queue thread is never
// started in unit tests, so these are never executed. Defining them here
// keeps the link independent of static archive ordering.
int px4_sem_init(px4_sem_t * /*s*/, int /*pshared*/, unsigned /*value*/) { return 0; }
int px4_sem_setprotocol(px4_sem_t * /*s*/, int /*protocol*/) { return 0; }
int px4_sem_wait(px4_sem_t * /*s*/) { return 0; }
int px4_sem_trywait(px4_sem_t * /*s*/) { return 0; }
int px4_sem_timedwait(px4_sem_t * /*s*/, const struct timespec * /*abstime*/) { return 0; }
int px4_sem_post(px4_sem_t * /*s*/) { return 0; }
int px4_sem_getvalue(px4_sem_t * /*s*/, int *sval) { if (sval) { *sval = 0; } return 0; }
int px4_sem_destroy(px4_sem_t * /*s*/) { return 0; }
