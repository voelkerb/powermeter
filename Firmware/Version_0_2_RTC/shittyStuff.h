

// Buffering stuff
// Number of bytes for one measurement
int MEASURMENT_BYTES = 8; //(16+2)
// Chunk sizes of data to send
const int PS_BUF_SIZE = 3*1024*1024;
#define NUM_RAM_BUFFERS 4
uint8_t* psdRamBuffer;
volatile uint32_t chunkSize = 256;
// Circular chunked buffer
// static uint8_t buffer[BUF_SIZE] = {0};
// Static vs dynamic allocation
#define MAX_SEND_SIZE 512 // 1024
static uint8_t buffer[NUM_RAM_BUFFERS][MAX_SEND_SIZE];
volatile bool flushBuffer[NUM_RAM_BUFFERS] = { false};
volatile uint8_t currentBuffer = 0;
// void createPSRAM_TASK() {
//   xTaskCreatePinnedToCore(
//         loop2, /* Function to implement the task */
//         "Loop2", /* Name of the task */
//         10000,  /* Stack size in words */
//         NULL,  /* Task input parameter */
//         1,  /* Priority of the task */
//         &TaskLoop2,  /* Task handle. */
//         0); /* Core where the task should run */
// }
//
// void loop2( void * pvParameters ) {
//   while (true) {
//     /*
//     for(uint8_t a = 0; a < NUM_RAM_BUFFERS; a++){
//       if(flushBuffer[a]) {
//         Serial.print("RW Distance:");
//         Serial.println(getReadWriteDistance());
//         // move buffer to PSDRAM
//         uint32_t end = (psdWritePtr+chunkSize)%PS_BUF_SIZE;
//
//         //memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][0], chunkSize);
//
//         if (end > psdWritePtr) {
//           memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][0], chunkSize);
//         } else {
//           memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][0], PS_BUF_SIZE - psdWritePtr);
//           memcpy((uint8_t*)&psdRamBuffer[psdWritePtr],(uint8_t*)&buffer[a][PS_BUF_SIZE - psdWritePtr], end);
//         }
//         psdWritePtr = end;
//         flushBuffer[a] = false;
//       }
//     }*/
//     vTaskDelay(2);
//     yield();
//   }
// }


// Copy into RAM buffer here because we cannot use PSRAM in interrupt
//memcpy(&buffer[currentBuffer][writePtr], (void*)&values[0], MEASURMENT_BYTES);
// Use psram in interrupt
ringBuffer.write(&values[0], MEASURMENT_BYTES);
memcpy(&psdRamBuffer[psdWritePtr], (void*)&values[0], MEASURMENT_BYTES);
psdWritePtr = (psdWritePtr+MEASURMENT_BYTES)%PS_BUF_SIZE;

// writePtr = writePtr+MEASURMENT_BYTES;
// if (writePtr >= chunkSize) {
//   flushBuffer[currentBuffer] = true;
//   writePtr = 0;
//   currentBuffer = (currentBuffer+1)%NUM_RAM_BUFFERS;
//   if(flushBuffer[currentBuffer]) {
//     Serial.println(F("Info:BufferOvf"));
//     packetNumber += NUM_RAM_BUFFERS;
//     currentBuffer = (currentBuffer+1)%NUM_RAM_BUFFERS;
//   }
// }
