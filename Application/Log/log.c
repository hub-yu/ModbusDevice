#include "LOG.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// #define PRINT_DATETIME
#ifdef PRINT_DATETIME
#include "rtc.h"
#endif

#ifdef OS
#include "os.h"
#endif

static char printBuf[100];
static const char* LevelString[LOG_LEVEL_ALL] = {
    "[NONE] ",
    "[ERROR] ",
    "[WARN] ",
    "[INFO] ",
    "[DEBUG] ",
};

#ifdef OS
log_level_enum log_level = LOG_LEVEL_INFO;
#else
log_level_enum log_level = LOG_LEVEL_ALL;
#endif


log_hook loghook = NULL;
// List* listPrintFun = ((void *)0);

static uint8_t str[1024];

uint8_t* ByteArrayToStr(const uint8_t* array, int32_t arrayLen) {
#ifdef OS
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
#endif
    if (arrayLen > 1000)
        return NULL;

    uint8_t highByte, lowByte;
    for (int32_t i = 0; (i < arrayLen); i++) {
        highByte = array[i] >> 4;
        lowByte = array[i] & 0x0f;

        highByte += 0x30;
        if (highByte > 0x39)
            str[i * 3] = highByte + 0x07;
        else
            str[i * 3] = highByte;

        lowByte += 0x30;
        if (lowByte > 0x39)
            str[i * 3 + 1] = lowByte + 0x07;
        else
            str[i * 3 + 1] = lowByte;

        str[i * 3 + 2] = 0x20;
    }
    str[arrayLen * 3] = 0;
#ifdef OS
    CPU_CRITICAL_EXIT();
#endif
    return str;
}

void Print(log_level_enum level, char* fmt, ...) {
    // return;
#ifdef OS
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();
#endif
    if (level > log_level) {
#ifdef OS
        CPU_CRITICAL_EXIT();
#endif
        return;
    }

    va_list va;


#ifdef PRINT_DATETIME

    uint32_t sec, usec;
    RtcGetRunTime(&sec, &usec);

    memset(printBuf, 0, 2048);
    sprintf(printBuf, "|%ld.%06ld| %s", sec, usec, LevelString[level]);
    //  struct tm t;
    //     uint32_t usec;
    //     RtcGetDateTime(&t, &usec);

    // sprintf(printBuf, "|%04d.%02d.%02d-%02d:%02d:%02d.%06ld| %s",
    //         t.tm_year + 1900,
    //         t.tm_mon + 1,
    //         t.tm_mday,
    //         t.tm_hour, t.tm_min, t.tm_sec, usec,
    //         LevelString[level]);
#else
    sprintf(printBuf, "%s ", LevelString[level]);
#endif

   
    va_start(va, fmt);
#ifdef PRINT_DATETIME
    // vsnprintf(printBuf + 29 + strlen(LevelString[level]), 200, (const char *)fmt, va);
    // vsnprintf(printBuf + 18 + strlen(LevelString[level]), 200, (const char *)fmt, va);
    vsnprintf(printBuf + strlen(printBuf), 200, (const char*)fmt, va);
#else
    vsnprintf(printBuf + strlen(LevelString[level]), 90, (const char*)fmt, va);
#endif
    va_end(va);

    if (loghook)
        loghook((const uint8_t*)printBuf, strlen(printBuf));
#ifdef OS
    CPU_CRITICAL_EXIT();
#endif
    // if(listPrintFun) {
    // 	List* node = listPrintFun->next;
    // 	while(node) {
    // 		((PrintFun)(node->element))((const uint8_t*)printBuf, strlen(printBuf));
    // 		node = node->next;
    // 	}
    // }
}

inline void LogSetLevel(log_level_enum level) {
    log_level = level;
}

void LogSetHook(const log_hook hook) {
    loghook = hook;
}