#include <dummy.h>

/*
  TimeLapseAvi
  ESP32-CAM Video Recorder
  This program records an AVI video on the SD Card of an ESP32-CAM.
  by James Zahary July 20, 2019  TimeLapseAvi23x.ino
     jamzah.plc@gmail.com
  https://github.com/jameszah/ESP32-CAM-Video-Recorder
    jameszah/ESP32-CAM-Video-Recorder is licensed under the
    GNU General Public License v3.0
  The is Arduino code, with standard setup for ESP32-CAM
    - Board ESP32 Wrover Module
    - Partition Scheme Huge APP (3MB No OTA)
*/

// 1 for blink red led with every sd card write, at your frame rate
// 0 for blink only for skipping frames and SOS if camera or sd is broken
//#define BlinkWithWrite 0
#define BLINK_ON_WRITE

// startup defaults for first recording

// here are the recording options from the "retart web page" 
// VGA 10 fps for 30 min, repeat, realtime                   http://192.168.0.117/start?framesize=VGA&length=1800&interval=100&quality=10&repeat=100&speed=1&gray=0
// VGA 2 fps, for 30 minutes repeat, 30x playback            http://192.168.0.117/start?framesize=VGA&length=1800&interval=500&quality=10&repeat=300&speed=30&gray=0
// UXGA 1 sec per frame, for 30 minutes repeat, 30x playback http://192.168.0.117/start?framesize=UXGA&length=1800&interval=1000&quality=10&repeat=100&speed=30&gray=0
// UXGA 2 fps for 30 minutes repeat, 15x playback            http://192.168.0.117/start?framesize=UXGA&length=1800&interval=500&quality=10&repeat=100&speed=30&gray=0
// CIF 20 fps second for 30 minutes repeat                   http://192.168.0.117/start?framesize=CIF&length=1800&interval=50&quality=10&repeat=100&speed=1&gray=0

// reboot startup parameters here

//int  framesize = 6;                 // vga  (10 UXGA, 7 SVGA, 6 VGA, 5 CIF)
bool repeat_recording = true;         // record again when done with a file
int  xspeed = 1;                      // 1x playback speed (realtime is 1)
//int  quality = 10;                  // 10 on the 0..64 scale, or 10..50 subscale - 10 is good, 20 is grainy and smaller files
int  capture_interval = 100;          // 100 ms or 10 frames per second
int  max_frames = 18000;            // 18000 frames = 10 fps * 60 seconds * 30 minutes = half hour

#define RED_LED 33
#define WHITE_LED 4


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int  xlength = max_frames * capture_interval / 1000;
bool recording = false;
bool ready = false;

#include "esp_camera.h"

// Time
#include "time.h"

// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <SD_MMC.h>

// ble
#include <BLEDevice.h>

long current_millis;
long last_capture_millis = 0;

char *filename ;
char *stream ;
bool newfile = false;
int frames = 0;
FILE *myfile;
long bp;
long ap;
long bw;
long aw;
long totalp;
long totalw;
float avgp;
float avgw;
int overtime_count = 0;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

char str[20];
uint16_t n;
#define BUFFSIZE 512
uint8_t buf[BUFFSIZE];

static int i = 0;
uint8_t temp = 0, temp_last = 0;
unsigned long fileposition = 0;
uint16_t frame_cnt = 0;
uint16_t remnant = 0;
uint32_t length = 0;
uint32_t start_ms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;
bool is_header = false;
long bigdelta = 0;
bool cam_cpu_active = false;
int skipping = 0;
int skipped = 0;

int fb_max = 7;

camera_fb_t * fb_q[30];
int fb_in = 0;
int fb_out = 0;

camera_fb_t * fb = NULL;

FILE *avifile = NULL;
FILE *idxfile = NULL;


unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
unsigned long idx_offset = 0;

uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t   dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"

uint8_t  vga_w[2] = {0x80, 0x02}; // 640
uint8_t  vga_h[2] = {0xE0, 0x01}; // 480
uint8_t  cif_w[2] = {0x90, 0x01}; // 400
uint8_t  cif_h[2] = {0x28, 0x01}; // 296
uint8_t svga_w[2] = {0x20, 0x03}; // 800
uint8_t svga_h[2] = {0x58, 0x02}; // 600
uint8_t uxga_w[2] = {0x40, 0x06}; // 1600
uint8_t uxga_h[2] = {0xB0, 0x04}; // 1200

#define AVIOFFSET 240 // AVI main header length

const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, // 'RIFF'
  0xD8, 0x01, 0x0E, 0x00, // fileSize (3,623,947,776 bytes for following section)
  0x41, 0x56, 0x49, 0x20, // fileType 'AVI '
  
  0x4C, 0x49, 0x53, 0x54, // 'LIST'
  0xD0, 0x00, 0x00, 0x00, // listSize (3,489,660,928 bytes for following section)
  0x68, 0x64, 0x72, 0x6C, // listType 'hdrl'
  0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x36, 0x30, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// AviWriterTask runs on cpu 1 to write the avi file
//
TaskHandle_t CameraTask, AviWriterTask;
SemaphoreHandle_t baton;
int counter = 0;

// Forward declarations
void make_avi();
void start_avi();
void another_save_avi();
void end_avi();

void major_fail();
void init_camera();
void init_sdcard();
void do_fb();

void codeForAviWriterTask(void *parameter)
{
  for (;;) {
    if (ready) {
      make_avi();
    }
    // https://github.com/espressif/arduino-esp32/issues/3871
    // https://github.com/espressif/arduino-esp32/issues/903#issuecomment-349676959
    delay(1); // FreeRtos treats this delay(1) as a yield call
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// CameraTask runs on cpu 0 to take pictures and drop them in a queue
//
// Semaphores are being used because fb_q is being shared across both 
// async tasks
void codeForCameraTask(void *parameter)
{
  for (;;) { // infinite loop

    if (cam_cpu_active) {
      current_millis = millis();
      if (current_millis - last_capture_millis > capture_interval) {

        last_capture_millis = millis();

        /**************
         *    LOCK
         */
        xSemaphoreTake( baton, portMAX_DELAY );

        if (((fb_in + fb_max - fb_out) % fb_max) + 1 == fb_max) {
          xSemaphoreGive( baton );
          /**************
           *   UNLOCK
           */

          Serial.print(F(" Queue Full, Skipping ... "));  // the queue is full
          skipped++;
          skipping = 1;
        }

        if (skipping > 0 ) {

          #ifdef BLINK_ON_WRITE
            digitalWrite(RED_LED, LOW);
          #endif

          if (skipping % 2 == 0) {  // skip every other frame until queue is cleared

            frames = frames + 1;
            frame_cnt++;

            fb_in = (fb_in + 1) % fb_max;
            bp = millis();
            fb_q[fb_in] = esp_camera_fb_get();
            totalp = totalp - bp + millis();

          } else {
            Serial.print(((fb_in + fb_max - fb_out) % fb_max));  // skip an extra frame to empty the queue
            skipped++;
          }
          skipping = skipping + 1;
          if (((fb_in + fb_max - fb_out) % fb_max) == 0 ) {
            skipping = 0;
            Serial.println(F(" Queue cleared. "));
          }

          xSemaphoreGive( baton );
          /**************
           *   UNLOCK
           */

        } else {

          skipping = 0;
          frames = frames + 1;
          frame_cnt++;

          fb_in = (fb_in + 1) % fb_max;
          bp = millis();
          fb_q[fb_in] = esp_camera_fb_get();
          totalp = totalp - bp + millis();
          xSemaphoreGive( baton );
          /**************
           *   UNLOCK
           */

        }
      }
    }
    delay(1); // yield
  }
}


//
// Writes an uint32_t in Big Endian at current file position
//
static void inline print_quartet(unsigned long i, FILE * fd)
{
  uint8_t x[1]; // ptr trick

  x[0] = i % 0x100;
  fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  fwrite(x , 1, 1, fd);
}

// brownout detector
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// setup() runs on cpu 1
//

void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  // creates other problems

  Serial.begin(115200);

  Serial.setDebugOutput(true);

  // zzz
  Serial.println();
  Serial.println(F("-------------------------------------"));
  Serial.println(F("ESP-CAM AVI Recorder\n"));
  Serial.println(F("-------------------------------------"));

  pinMode(RED_LED, OUTPUT);    // little red led on back of chip
  digitalWrite(RED_LED, LOW);           // turn on the red LED on the back of chip

  if (!psramFound()) {
    Serial.println(F("no PSRAM found"));
    major_fail();
  }

  // SD camera init
  init_sdcard();

  digitalWrite(RED_LED, HIGH);         // Turn off red LED

  baton = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    codeForCameraTask,  //callback function
    "CameraTask",       //thread name
    10000,              //thread stack size
    NULL,               //function parameter
    1,                  //priority
    &CameraTask,        //reference handle 
    0);                 //cpu core

  delay(50);

  xTaskCreatePinnedToCore(
    codeForAviWriterTask,
    "AviWriterTask",
    10000,
    NULL,
    2,
    &AviWriterTask,
    1);

  delay(50);

  // ESP camera init
  init_camera();

  pinMode(WHITE_LED, OUTPUT);                 // using 1 bit mode, shut off the Blinding Disk-Active Light
  digitalWrite(WHITE_LED, LOW);

  // Let threads know to begin
  ready = true;

}


//
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//
void major_fail() {

  Serial.println(" ");

  for (int i = 0;  i < 10; i++) {                 // 10 loops or about 100 seconds then reboot
    digitalWrite(RED_LED, LOW);   delay(150);
    digitalWrite(RED_LED, HIGH);  delay(150);
    digitalWrite(RED_LED, LOW);   delay(150);
    digitalWrite(RED_LED, HIGH);  delay(150);
    digitalWrite(RED_LED, LOW);   delay(150);
    digitalWrite(RED_LED, HIGH);  delay(150);

    delay(1000);

    digitalWrite(RED_LED, LOW);  delay(500);
    digitalWrite(RED_LED, HIGH); delay(500);
    digitalWrite(RED_LED, LOW);  delay(500);
    digitalWrite(RED_LED, HIGH); delay(500);
    digitalWrite(RED_LED, LOW);  delay(500);
    digitalWrite(RED_LED, HIGH); delay(500);

    delay(1000);
    Serial.print(F("Major Fail  ")); Serial.print(i); Serial.print(F(" / ")); Serial.println(10);
  }

  ESP.restart();

}

void init_sdcard()
{
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;                       // using 1 bit mode
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 1;                                   // using 1 bit mode
  //Serial.print("Slot config width should be 4 width:  "); Serial.println(slot_config.width);
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
  };

  //pinMode(4, OUTPUT);                 // using 1 bit mode, shut off the Blinding Disk-Active Light
  //digitalWrite(4, LOW);

  sdmmc_card_t *card;

  Serial.println(F("Mounting SD card..."));
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    Serial.println(F("SD card mount successfully!"));
  }  else  {
    Serial.print(F("Failed to mount SD card VFAT filesystem. Error: "));
    Serial.print(esp_err_to_name(ret));
    major_fail();
  }
  sdmmc_card_print_info(stdout, card);
  //Serial.print(F("SD_MMC Begin: ")); Serial.println(SD_MMC.begin());   // required by ftp system ??

  if (!SD_MMC.begin()) {
    Serial.println("SD_MMC failed to begin");
    major_fail();
  }

  Serial.print(F("SD_MMC.usedBytes(): "));
  Serial.println(SD_MMC.usedBytes());

  Serial.print(F("SD_MMC.cardSize(): "));
  Serial.println(SD_MMC.cardSize());

  Serial.print(F("SD_MMC.totalBytes(): "));
  Serial.println(SD_MMC.totalBytes());  

  Serial.printf("Using %lluMB of %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024), SD_MMC.totalBytes() / (1024 * 1024));    // available bytes
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Make the avi move in 4 pieces
//
// make_avi() called in every loop, which calls below, depending on conditions
//   start_avi() - open the file and write headers
//   another_pic_avi() - write one more frame of movie
//   end_avi() - write the final parameters and close the file

void make_avi() {
  if (!newfile) {                             // open the file

    if (recording) {
      digitalWrite(RED_LED, HIGH);            // Disable red LED
      start_avi();
    }

  } else {

    if (!recording) {
      Serial.println(F("Stop recording request"));
      end_avi();
    } else if (frames >= max_frames ||        // max frames recorded
      millis() - start_ms > max_frames * capture_interval)  {
      
      Serial.println(F("Max frames reached"));

      end_avi();
      recording = repeat_recording;           // repeat a new recording
    }
    else  {                                   // continue recording
      another_save_avi();
    }    
  }
}

void init_camera() {

  camera_config_t config;

  //Serial.println("config camera");

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_VGA; //FRAMESIZE_UXGA;

  config.jpeg_quality = 10;
  config.fb_count = fb_max + 1;

  // camera init
  auto cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    Serial.print(F("Camera init failed with error 0x"));
    Serial.println(cam_err);
    major_fail();
  }

  delay(100);

  for (int j = 0; j < 3; j++) {
    do_fb();  // start the camera ... warm it up
    delay(1);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// start_avi - open the files and write in headers
//

void start_avi() {

  Serial.println(F("Starting an avi "));

  char fname[100];

  // while the file already exists, 
  // keep looking for new names
  for (;;) {  
    int rand = random(65536);
    sprintf(fname, "/sdcard/%d.avi", rand);
    if (!SD_MMC.exists(fname)) {
      // then break and open file
      break;
    }
  }

  Serial.print(F("\nFile name will be <"));
  Serial.print(fname);
  Serial.println(">");

  avifile = fopen(fname, "w");
  idxfile = fopen("/sdcard/idx.tmp", "w");

  if (!avifile) {
    Serial.println(F("Could not open AVI file"));
    major_fail();
  }

  if (!idxfile) {
    Serial.println(F("Could not open IDX file"));
    major_fail();
  }


  for ( i = 0; i < AVIOFFSET; i++)
  {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }

  fwrite(buf, 1, AVIOFFSET, avifile);

  sensor_t * s = esp_camera_sensor_get();


  switch (s->status.framesize) {
    case FRAMESIZE_CIF: {
      fseek(avifile, 0x40, SEEK_SET);
      fwrite(cif_w, 1, 2, avifile);
      fseek(avifile, 0xA8, SEEK_SET);
      fwrite(cif_w, 1, 2, avifile);
      fseek(avifile, 0x44, SEEK_SET);
      fwrite(cif_h, 1, 2, avifile);
      fseek(avifile, 0xAC, SEEK_SET);
      fwrite(cif_h, 1, 2, avifile);
      break;
    } 
    case FRAMESIZE_VGA: {
      fseek(avifile, 0x40, SEEK_SET);
      fwrite(vga_w, 1, 2, avifile);
      fseek(avifile, 0xA8, SEEK_SET);
      fwrite(vga_w, 1, 2, avifile);
      fseek(avifile, 0x44, SEEK_SET);
      fwrite(vga_h, 1, 2, avifile);
      fseek(avifile, 0xAC, SEEK_SET);
      fwrite(vga_h, 1, 2, avifile);
      break;
    } 
    case FRAMESIZE_SVGA: {
      fseek(avifile, 0x40, SEEK_SET);
      fwrite(svga_w, 1, 2, avifile);
      fseek(avifile, 0xA8, SEEK_SET);
      fwrite(svga_w, 1, 2, avifile);
      fseek(avifile, 0x44, SEEK_SET);
      fwrite(svga_h, 1, 2, avifile);
      fseek(avifile, 0xAC, SEEK_SET);
      fwrite(svga_h, 1, 2, avifile);
      break;
    } 
    case FRAMESIZE_UXGA: {
      fseek(avifile, 0x40, SEEK_SET);
      fwrite(uxga_w, 1, 2, avifile);
      fseek(avifile, 0xA8, SEEK_SET);
      fwrite(uxga_w, 1, 2, avifile);
      fseek(avifile, 0x44, SEEK_SET);
      fwrite(uxga_h, 1, 2, avifile);
      fseek(avifile, 0xAC, SEEK_SET);
      fwrite(uxga_h, 1, 2, avifile);
      break;
    } default: {
      Serial.println(F("Unknown framesize"));
      major_fail();
    }
  }

  fseek(avifile, AVIOFFSET, SEEK_SET);

  Serial.print(F("\nRecording "));
  Serial.print(max_frames);
  Serial.println(F(" video frames ..."));
  Serial.println();

  start_ms = millis();
  bigdelta = millis();
  totalp = 0;
  totalw = 0;
  overtime_count = 0;
  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;


  frame_cnt = 0;
  frames = 0;

  skipping = 0;
  skipped = 0;

  newfile = true;

  cam_cpu_active = true; // Start taking pictures

} // end of start avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  another_save_avi runs on cpu 1, saves another frame to the avi file
//
//  the "baton" semaphore makes sure that only one cpu is using the camera subsystem at a time
//

void another_save_avi() {

  // Protect the shared resources
  xSemaphoreTake( baton, portMAX_DELAY );

  if (fb_in == fb_out) {        // nothing to do

    xSemaphoreGive( baton );

  } else {

    Serial.print(F("writing frame "));
    Serial.println(frames);

    fb_out = (fb_out + 1) % fb_max;

    int fblen;
    fblen = fb_q[fb_out]->len;

    //xSemaphoreGive( baton );

    #ifdef BLINK_ON_WRITE
      digitalWrite(RED_LED, LOW);
    #endif

    jpeg_size = fblen;
    movi_size += jpeg_size;
    uVideoLen += jpeg_size;

    bw = millis();
    fwrite(dc_buf, 1, 4, avifile);
    fwrite(zero_buf, 1, 4, avifile);

    bw = millis();
    size_t err = fwrite(fb_q[fb_out]->buf, 1, fb_q[fb_out]->len, avifile);
    if (err == 0 ) {
      Serial.println(F("Error on avi fb write"));
      major_fail();
    }
    totalw = totalw + millis() - bw;

    esp_camera_fb_return(fb_q[fb_out]);     // frame no longer needed
    xSemaphoreGive( baton );                // fb_q access can be opened

    remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

    print_quartet(idx_offset, idxfile);
    print_quartet(jpeg_size, idxfile);

    idx_offset = idx_offset + jpeg_size + remnant + 8;

    jpeg_size = jpeg_size + remnant;
    movi_size = movi_size + remnant;
    if (remnant > 0) {
      fwrite(zero_buf, 1, remnant, avifile);
    }

    fileposition = ftell (avifile);       // Here, we are at end of chunk (after padding)
    fseek(avifile, fileposition - jpeg_size - 4, SEEK_SET);    // Here we are the the 4-bytes blank placeholder

    print_quartet(jpeg_size, avifile);    // Overwrite placeholder with actual frame size (without padding)

    fileposition = ftell (avifile);

    fseek(avifile, fileposition + 6, SEEK_SET);    // Here is the FOURCC "JFIF" (JPEG header)
    // Overwrite "JFIF" (still images) with more appropriate "AVI1"

    fwrite(avi1_buf, 1, 4, avifile);

    fileposition = ftell (avifile);
    fseek(avifile, fileposition + jpeg_size - 10 , SEEK_SET);
    //Serial.println("Write done");
    //41 totalw = totalw + millis() - bw;

    //if (((fb_in + fb_max - fb_out) % fb_max) > 0 ) {
    //  Serial.print(((fb_in + fb_max - fb_out) % fb_max)); Serial.print(" ");
    //}

    digitalWrite(RED_LED, HIGH);          // Turn off red LED
  }
} // end of another_pic_avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  end_avi runs on cpu 1, empties the queue of frames, writes the index, and closes the files
//

void end_avi() {

  Serial.println(F("Stopping recording..."));

  digitalWrite(RED_LED, LOW); // Turn off red LED

  unsigned long current_end = 0;

  cam_cpu_active = false;  // shuts down the picture taking program

  //Serial.print(" Write Q: "); Serial.print((fb_in + fb_max - fb_out) % fb_max); Serial.print(" in/out  "); Serial.print(fb_in); Serial.print(" / "); Serial.println(fb_out);

  for (int i = 0; i < fb_max; i++) {           // clear the queue
    another_save_avi();
  }

  //Serial.print(" Write Q: "); Serial.print((fb_in + fb_max - fb_out) % fb_max); Serial.print(" in/out  "); Serial.print(fb_in); Serial.print(" / "); Serial.println(fb_out);

  current_end = ftell (avifile);

  //Serial.println(F("End of avi - closing the files"));

  elapsedms = millis() - start_ms;
  float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * xspeed;
  float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
  uint8_t iAttainedFPS = round(fRealFPS);
  uint32_t us_per_frame = round(fmicroseconds_per_frame);


  //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

  fseek(avifile, 4 , SEEK_SET);
  print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

  fseek(avifile, 0x20 , SEEK_SET);
  print_quartet(us_per_frame, avifile);

  unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt;

  fseek(avifile, 0x24 , SEEK_SET);
  print_quartet(max_bytes_per_sec, avifile);

  fseek(avifile, 0x30 , SEEK_SET);
  print_quartet(frame_cnt, avifile);

  fseek(avifile, 0x8c , SEEK_SET);
  print_quartet(frame_cnt, avifile);

  fseek(avifile, 0x84 , SEEK_SET);
  print_quartet((int)iAttainedFPS, avifile);

  fseek(avifile, 0xe8 , SEEK_SET);
  print_quartet(movi_size + frame_cnt * 8 + 4, avifile);

  Serial.println(F("\n*** Video recorded and saved ***\n"));
  Serial.print(F("Recorded "));
  Serial.print(elapsedms / 1000);
  Serial.print(F("s in "));
  Serial.print(frame_cnt);
  Serial.print(F(" frames\nFile size is "));
  Serial.print(movi_size + 12 * frame_cnt + 4);
  Serial.print(F(" bytes\nActual FPS is "));
  Serial.print(fRealFPS, 2);
  Serial.print(F("\nMax data rate is "));
  Serial.print(max_bytes_per_sec);
  Serial.print(F(" byte/s\nFrame duration is "));  Serial.print(us_per_frame);  Serial.println(F(" us"));
  Serial.print(F("Average frame length is "));  Serial.print(uVideoLen / frame_cnt);  Serial.println(F(" bytes"));
  Serial.print(F("Average picture time (ms) ")); Serial.println( totalp / frame_cnt );
  Serial.print(F("Average write time (ms)   ")); Serial.println( totalw / frame_cnt );
  Serial.print(F("Frames Skipped % "));  Serial.println( 100.0 * skipped / max_frames, 1 );

  Serial.println(F("Writing the index"));

  fseek(avifile, current_end, SEEK_SET);

  fclose(idxfile);

  fwrite(idx1_buf, 1, 4, avifile);

  print_quartet(frame_cnt * 16, avifile);

  idxfile = fopen("/sdcard/idx.tmp", "r");
  if (!idxfile) {
    Serial.println(F("Could not open IDX file"));
    major_fail();
  }

  //char * AteBytes;
  //AteBytes = (char*) malloc (8);

  char AteBytes[8];

  for (int i = 0; i < frame_cnt; i++) {
    fread (AteBytes,  1, 8, idxfile);
    fwrite(dc_buf,    1, 4, avifile);
    fwrite(zero_buf,  1, 4, avifile);
    fwrite(AteBytes,  1, 8, avifile);
  }

  //free(AteBytes);
  fclose(idxfile);
  fclose(avifile);
  remove("/sdcard/idx.tmp");

  Serial.println("---");

  frames = 0;
  recording = false;
  newfile = false;

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  do_fb - just takes a picture and discards it
//

void do_fb() {
  xSemaphoreTake(baton, portMAX_DELAY);
  camera_fb_t * fb = esp_camera_fb_get();

  Serial.print(F("Pic, len=")); Serial.println(fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive(baton);
}

void loop()
{
  if (Serial.available()) {
    //int incoming = Serial.read();
    String s = Serial.readString();

    if (s == F("rec")) {
      recording = true;
    } else if (s == F("stop")) {
      recording = false;
    }
  }
  //Serial.println("This code is called from loop()");
  delay(500);
}