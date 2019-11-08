

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


char buf2[1000];

//This function gathers the stack itself from starter to ender
void getStack(uint32_t starter, uint32_t ender){
  char stackline[46];

  for (uint32_t pos = starter; pos < ender; pos += 0x10) {
      uint32_t* values = (uint32_t*)(pos);
      //rough indicator: stack frames usually have SP saved as the second word
      bool looksLikeStackFrame = (values[2] == pos + 0x10);
      sprintf(stackline, "%08x:  %08x %08x %08x %08x %c", pos, values[0], values[1], values[2], values[3], (looksLikeStackFrame)?'<':' ');
      sprintf(buf2 + strlen(buf2), "%s", stackline);
  } 
}

extern "C" void custom_crash_callback(struct rst_info * rst_info, uint32_t stack, uint32_t stack_end ){  
  register uint32_t sp asm("a1");
  cont_t g_cont __attribute__ ((aligned (16)));

  uint32_t cont_stack_start = (uint32_t) &(g_cont.stack);
  uint32_t cont_stack_end = (uint32_t) g_cont.stack_end;
  uint32_t stack_end2 = stack_end;
  uint32_t offset = 0;

  if (rst_info->reason == REASON_SOFT_WDT_RST) {
    offset = 0x1b0;
  } else if (rst_info->reason == REASON_EXCEPTION_RST) {
    offset = 0x1a0;
  } else if (rst_info->reason == REASON_WDT_RST) {
    offset = 0x10;
  } 
  
  if (stack > cont_stack_start && stack < cont_stack_end) {
    sprintf(buf2 + strlen(buf2), "%s", "ctx: cont");
  } else {
    sprintf(buf2 + strlen(buf2), "%s", "ctx: sys");
  }
  sprintf(buf2 + strlen(buf2), "sp: %08x end: %08x offset: %04x\n", stack, stack_end, offset);
  getStack(stack, stack_end);

  eeprom_erase_all();
  eeprom_write_string(0, buf2);
  EEPROM.commit();
  Serial.println("Oh my good, reset happened...");
} 