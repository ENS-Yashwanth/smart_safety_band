#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void voice_call_task_start(void);
/* Invoke from the confirmed local cancel/override UI action. */
void voice_call_task_cancel(void);

#ifdef __cplusplus
}
#endif
