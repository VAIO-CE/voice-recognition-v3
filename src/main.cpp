#include "VoiceRecognitionV3.h"

// Function declarations
void start_vr3_running(void);
boolean byte_array_cmp(uint8_t *a, uint8_t *b, int len_a, int len_b);
boolean check_for_vr3_load_response(int timeout);
void voice_recognition_task(void *pvParameters); // FreeRTOS task for voice recognition

VR myVR(16, 17); // Voice recognition module on pins 16 and 17

SemaphoreHandle_t xSemaphore; // Declare a semaphore handle

// The following arrays are used for the Elechouse VR3 module
uint8_t vr3_check_recog_cmd[]  = {0x01};
uint8_t vr3_clear_recog_cmd[]  = {0x31};
uint8_t vr3_load_records_cmd[] = {0x30, 0x00, 0x01, 0x02, 0x03};
uint8_t vr3_load_response[]    = {0xAA, 0x09, 0x30, 0x03, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x0A};

uint8_t vr3_move1_msg[] = {0xAA, 0x0B, 0x0D, 0x00, 0xFF, 0x01, 0x01, 0x04, 0x4D, 0x4F, 0x56, 0x45, 0x0A};
uint8_t vr3_move2_msg[] = {0xAA, 0x0B, 0x0D, 0x00, 0xFF, 0x04, 0x04, 0x04, 0x4D, 0x4F, 0x56, 0x45, 0x0A};

uint8_t vr3_control1_msg[] = {0xAA, 0x0E, 0x0D, 0x00, 0xFF, 0x02, 0x02, 0x07, 0x43, 0x4F, 0x4E, 0x54, 0x52, 0x4F, 0x4C, 0x0A};
uint8_t vr3_control2_msg[] = {0xAA, 0x0E, 0x0D, 0x00, 0xFF, 0x05, 0x05, 0x07, 0x43, 0x4F, 0x4E, 0x54, 0x52, 0x4F, 0x4C, 0x0A};

uint8_t vr3_hand1_msg[] = {0xAA, 0x0B, 0x0D, 0x00, 0xFF, 0x03, 0x03, 0x04, 0x48, 0x41, 0x4E, 0x44, 0x0A};
uint8_t vr3_hand2_msg[] = {0xAA, 0x0B, 0x0D, 0x00, 0xFF, 0x06, 0x06, 0x04, 0x48, 0x41, 0x4E, 0x44, 0x0A};

uint8_t vr3_test_msg[] = {0xAA, 0x0B, 0x0D, 0x00, 0xFF, 0x00, 0x00, 0x04, 0x54, 0x45, 0x53, 0x54, 0x0A};
uint8_t vr3_buf[50];

void setup(void){
  Serial.begin(115200);
  xSemaphore = xSemaphoreCreateBinary();

  // Start the VR3 running and signal that it is ready
  start_vr3_running();
  xTaskCreatePinnedToCore(voice_recognition_task,"VoiceRecTask",4096,NULL,1,NULL,0);
}

void loop(){
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void voice_recognition_task(void *pvParameters){
  // Wait for the semaphore to be given before proceeding
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
    Serial.println("Voice recognition task started.");
    
    int ret_len;

    while (1) {
      ret_len = myVR.receive_pkt(vr3_buf, 50);
      if (ret_len > 0) {
        if (byte_array_cmp(vr3_buf, vr3_move1_msg, ret_len, sizeof(vr3_move1_msg)) ||
            byte_array_cmp(vr3_buf, vr3_move2_msg, ret_len, sizeof(vr3_move2_msg))) {
          Serial.println(F("Heard: move  "));
          // Other tasks to do
        }
        else if (byte_array_cmp(vr3_buf, vr3_control1_msg, ret_len, sizeof(vr3_control1_msg)) ||
                 byte_array_cmp(vr3_buf, vr3_control2_msg, ret_len, sizeof(vr3_control2_msg))) {
          Serial.println(F("Heard: control"));
          // Other tasks to do
        }
        else if (byte_array_cmp(vr3_buf, vr3_hand1_msg, ret_len, sizeof(vr3_hand1_msg)) ||
                 byte_array_cmp(vr3_buf, vr3_hand2_msg, ret_len, sizeof(vr3_hand2_msg))) {
          Serial.println(F("Heard: hand "));
          // Other tasks to do
        }
        else if (byte_array_cmp(vr3_buf, vr3_test_msg, ret_len, sizeof(vr3_test_msg))) {
          Serial.println(F("Heard: test "));
          // Other tasks to do
        }
        else {
          Serial.println(F("Unknown word"));
          // Other tasks to do
        }
      }

      vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
  }
}

// Starts the VR3 module running, by loading the records into the recognizer 
void start_vr3_running(void){
  myVR.begin(9600);

  // Check and clear recognizer, get responses, ignore them to get VR3 started in a known state
  myVR.receive_pkt(vr3_buf, 50);
  myVR.send_pkt(vr3_check_recog_cmd, sizeof(vr3_check_recog_cmd));
  myVR.receive_pkt(vr3_buf, 50);
  myVR.send_pkt(vr3_clear_recog_cmd, sizeof(vr3_clear_recog_cmd));
  myVR.receive_pkt(vr3_buf, 50);

  // Load word recognition records
  myVR.send_pkt(vr3_load_records_cmd, sizeof(vr3_load_records_cmd));

  if (check_for_vr3_load_response(1000))
    Serial.println("Listening...");
  else
    Serial.println("VR3 Not Started");

  // Give the semaphore to signal the voice recognition task that VR3 is ready
  xSemaphoreGive(xSemaphore);
}

// Returns true if the VR3 module returns the expected response to the "load records" command within the specified milliseconds timeout.
boolean check_for_vr3_load_response(int timeout){
  int ret_len;

  ret_len = myVR.receive_pkt(vr3_buf, timeout);
  Serial.println(ret_len);
  if (ret_len <= 0)
    return false;

  return byte_array_cmp(vr3_buf, vr3_load_response, ret_len, sizeof(vr3_load_response));
}

// Compares two arrays, returns true if they're identical
boolean byte_array_cmp(uint8_t *a, uint8_t *b, int len_a, int len_b){
  if (len_a != len_b)
    return false;

  for (int n = 0; n < len_a; n++)
    if (a[n] != b[n])
      return false;

  return true;
}
