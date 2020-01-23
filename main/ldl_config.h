#ifndef LDL_CONFIG_H_
#define LDL_CONFIG_H_

#include <stdio.h>
#include <assert.h>
#include <inttypes.h>

#define LDL_ENABLE_SX1276
#define LDL_ENABLE_EU_863_870

#define LDL_DISABLE_POINTONE
#define LDL_DISABLE_SESSION_UPDATE
#define LDL_ENABLE_STATIC_RX_BUFFER
#define LDL_DISABLE_CHECK
#define LDL_DISABLE_DEVICE_TIME
#define LDL_DISABLE_FULL_CHANNEL_CONFIG
#define LDL_DISABLE_CMD_DL_CHANNEL

#define LDL_ERROR(APP,...) do{fprintf(stderr,  "%u: %s: error: ", __LINE__, __FUNCTION__);fprintf(stderr, __VA_ARGS__);fprintf(stderr, "\n");}while(0);
#define LDL_DEBUG(APP,...) do{fprintf(stderr,  "%u: %s: debug: ", __LINE__, __FUNCTION__);fprintf(stderr, __VA_ARGS__);fprintf(stderr, "\n");}while(0);
#define LDL_INFO(APP,...) do{fprintf(stderr,   "%u: %s: info: ", __LINE__, __FUNCTION__);fprintf(stderr, __VA_ARGS__);fprintf(stderr, "\n");}while(0);
#define LDL_ASSERT(x) assert((x));
#define LDL_PEDANTIC(x) assert((x));

#endif /* LDL_CONFIG_H_ */
