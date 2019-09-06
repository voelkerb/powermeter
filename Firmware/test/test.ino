extern "C" {
  #include <esp_spiram.h>
  #include <esp_himem.h>
}


const int N = 3*1024*1024+1020;
byte* buff;

void setup() {
  Serial.begin(115200);
  esp_spiram_init();
  printf("spiram size %u\n", esp_spiram_get_size());
  printf("himem free %u\n", esp_himem_get_free_size());
  printf("himem phys %u\n", esp_himem_get_phys_size());
  printf("himem reserved %u\n", esp_himem_reserved_area_size());
  Serial.printf("spiram size %u\n", esp_spiram_get_size());
  // Free Memory
  Serial.printf("\n%d Bytes free.\n\n", ESP.getFreeHeap());
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
