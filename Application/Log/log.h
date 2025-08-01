#ifndef LOG_H
#define LOG_H

#include <stdint.h>

typedef enum {
	LOG_LEVEL_NONE,
	LOG_LEVEL_ERROR,
	LOG_LEVEL_WARN,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG,
	LOG_LEVEL_ALL,
} log_level_enum;

typedef void (*log_hook)(const uint8_t*, int32_t);

uint8_t* ByteArrayToStr(const uint8_t* array, int32_t arrayLen);
void Print(log_level_enum level, char* fmt, ...);

#define LOG_ERROR(msg, ...)		{Print(LOG_LEVEL_ERROR, msg"",##__VA_ARGS__);}
#define LOG_WARN(msg, ...)		{Print(LOG_LEVEL_WARN, msg"",##__VA_ARGS__);}
#define LOG_INFO(msg, ...)		{Print(LOG_LEVEL_INFO, msg"",##__VA_ARGS__);}
#define LOG_DEBUG(msg, ...)		{Print(LOG_LEVEL_DEBUG, msg"",##__VA_ARGS__);}


// #define LOG_ERROR(msg, ...)		{Print(LOG_LEVEL_ERROR, "[ %s %s %d ]: "msg"", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);}
// #define LOG_WARN(msg, ...)		{Print(LOG_LEVEL_WARN, "[ %s %s %d ]: "msg"", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);}
// #define LOG_INFO(msg, ...)		{Print(LOG_LEVEL_INFO, "[ %s %s %d ]: "msg"", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);}
// #define LOG_DEBUG(msg, ...)		{Print(LOG_LEVEL_DEBUG, "[ %s %s %d ]: "msg"", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);}

// #define Fll(msg, ...)		 	Print(LEVEL_NONE, "[ %s %s %d ]: "msg"", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);

void LogSetLevel(log_level_enum level);
void LogSetHook(const log_hook hock);

#endif /* LOG_H */