#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ESP32Encoder ESP32Encoder; // Forward declaration

// Wrapper function declarations
ESP32Encoder* createEncoder();
void attachHalfQuad(ESP32Encoder* encoder, int aPinNumber, int bPinNumber);
int64_t getEncoderCount(ESP32Encoder* encoder);
void deleteEncoder(ESP32Encoder* encoder);
int64_t clearCount(ESP32Encoder* encoder);  // Corrected declaration

#ifdef __cplusplus
}
#endif
