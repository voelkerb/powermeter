extern "C" {
  #include <esp_spiram.h>
  #include <esp_himem.h>
}


const int N = 3*1024*1024+1020;
byte* buff;

void setup() {
  Serial.begin(2000000);

  Serial.println("========================================");
  Serial.printf("PSRAM total size     : %u \n", esp_spiram_get_size());
  Serial.println("----------------------------------------");
  Serial.printf("PSRAM first 4MB size : %u \n", ESP.getPsramSize());
  Serial.printf("PSRAM first 4MB free : %u \n", ESP.getMaxAllocPsram());
  Serial.printf("PSRAM HI-MEM    size : %u \n", esp_himem_get_phys_size());
  Serial.printf("PSRAM HI-MEM    free : %u \n", esp_himem_get_free_size());
  Serial.println("========================================");
  Serial.printf("Internal RAM  size   : %u \n", ESP.getHeapSize());
  Serial.printf("Internal RAM  free   : %u \n", ESP.getFreeHeap());
  Serial.println("========================================");

  Serial.println("Testing the free memory of PSRAM HI-MEM ...");




  // ps_malloc
  if (buff = (byte*)ps_malloc(sizeof(byte)*N))
    Serial.println("ps_malloc succeeded");
  else
    Serial.println("ps_malloc failed");
    /*
  // heap_caps_malloc
  if (buff = (byte*)heap_caps_malloc(sizeof(byte)*N, MALLOC_CAP_8BIT))
    Serial.println("heap_caps_malloc succeeded");
  else
    Serial.println("heap_caps_malloc failed");
  // heap_info
  Serial.println("\nheap_info");
  heap_caps_print_heap_info(MALLOC_CAP_8BIT);
  Serial.println();*/


  // test SPI RAM
  Serial.print("Test 128B:"); testram(128);
  Serial.print("Test 256B:"); testram(256);
  Serial.print("Test  1KB:"); testram(1024);
  Serial.print("Test 16KB:"); testram(16*1024);
  Serial.print("Test 64KB:"); testram(64*1024);
  Serial.print("Test  1MB:"); testram(1*1024*1024);
  Serial.print("Test  4MB:"); testram(4*1024*1024);
}

void loop() {
}

void testram(int ramSize) {
  // Test SPI-RAM: write

  for(uint64_t i=0; i<(ramSize); i++) {
    buff[i] = (uint8_t)(i%255);
  }
  // Test SPI-RAM: verify
  int e = 0;
  for(uint64_t i=0; i<(ramSize); i++) {
    if(buff[i] != (uint8_t)(i%255)) e++;
  }
  Serial.printf(" %7d Bytes: %7d OK, %7d Errors.\n",ramSize,ramSize-e,e);
}
