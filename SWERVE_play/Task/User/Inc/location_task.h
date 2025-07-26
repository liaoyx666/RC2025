#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif
	
void Location_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif
