//#define DEBUG_OUT_FLAG
//#define USE_OFFSET

#include "model_data.h"
#include<TensorFlowLite.h>
#include<stdlib.h>
#include <Arduino_LSM9DS1.h>
#include<Wire.h>
#include<Arduino.h>
#include<ArduinoBLE.h>
#include<string.h>

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#define DATA_SAMPLE_COUNT 2
#define DATA_QUEUE_SIZE 6

#define IMU_SAMPLE_CHANNEL 6
#define IMU_SAMPLE_COUNT 30

#define COMMAND_BUFFER_SIZE 10
#define GESTURE_WINDOW_SIZE 20

#define COMMAND_LIST_SIZE 15
#define MAX_GESTURE_PER_COMMAND COMMAND_BUFFER_SIZE

#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3
#define UNKNOWN 4
#define SILENT 5

#define SIGNATURE_TRAIN_VALUE 2

#define BLE_LOCAL_NAME "MagicWand_BLE"
#define UUID(val) "7843" val "-91f7-46a1-8795-173154692a6a"
#define BASE "b000"
#define NOTIFY "b001"
#define SYNC "b002"

#define PAIR_BUTTON 3
#define GESTURE_BUTTON 10

#define LED_RED 22
#define LED_GREEN 23
#define LED_BLUE 24  // Arudino nano 33 ble sense's LED pin

#define ALPHA 0.95

#define CHURON_THRESHOLD -50 // 추론 임계치

#define ACCL_MULT 22.0
#define GYRO_MULT 0.5

#define HASH_VAL 0x811C9DC5
#define HKEY0 0x81
#define HKEY1 0x1C
#define HKEY2 0x9D
#define HKEY3 0xC5

float imu_data_buffer[IMU_SAMPLE_CHANNEL];
float imu_data_queue[IMU_SAMPLE_CHANNEL * IMU_SAMPLE_COUNT];

bool queue_is_full = 0;
int queue_last_pts = 0;

bool have_command = false;

uint8_t last_result = UNKNOWN;
uint8_t last_result_before = UNKNOWN;

bool before_button_status = 0;

uint8_t command_buffer[COMMAND_BUFFER_SIZE]; // 이건 커맨드 스택이므로 로테이션을 사용하지 않음
int command_buffer_occupied = 0;

uint8_t command_list[COMMAND_LIST_SIZE][MAX_GESTURE_PER_COMMAND+2] = {0,}; // [][0] -> is there command? [][1] -> Actual Command Length [][2-] -> Actual command
uint8_t command_list_size = 0;

float baselines[6] = {0,0,0,0,0,0};//잔차 연산을 위함.
const float MULT_ARRAY[6] = {15.0, 18.0, 18.0, 0.1, 0.15, 0.15}; //ax, ay, az, gx, gy, gz
const float MULT_OFFSET[6] = {36, -3, -21, 3, -17, -25 }; // MULT_ARRAY -> 축별 배율, MULT_OFFSET -> 오프셋
const float BASELINE_ALPHAS[6] = {0.93, 0.93, 0.93, 0.98, 0.98, 0.98};

float time_baselines[IMU_SAMPLE_CHANNEL * IMU_SAMPLE_COUNT] = {0,};

BLEDevice central;
//time_baselines follows queue_last_pts.
bool is_synced = false;



namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
int inference_count = 0;


// Create an area of memory to use for input, output, and intermediate arrays.
// Finding the minimum value for your model may require some trial and error.
constexpr int kTensorArenaSize = 16 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}  // namespace
uint32_t itime = 0;
uint32_t ttime = 0;

BLEService gestureService(UUID(BASE));
BLECharacteristic notifyCharacteristic(UUID(NOTIFY), BLERead | BLENotify, 1);
BLECharacteristic syncCharacteristic(UUID(SYNC), BLERead | BLEWrite | BLENotify, 20);

bool is_Synced = false;
uint8_t incoming_buffer[20];
uint8_t outgoing_buffer[20];

void handle_result(int8_t* results){
  int8_t max = -127;
  uint8_t idx = 0;
  for(int i=0;i<6;i++){ // 흔들림 무시하고 일단 최대값만 찾아서 삽입
    if(max<results[i]){
      max = results[i];
      idx = i;// 겹치면 유감
    }
  }

  if(max<CHURON_THRESHOLD)idx = UNKNOWN;
  last_result_before = last_result;
  last_result = idx;
}
uint8_t stResult = 0;
uint8_t before_status = UNKNOWN;
void handle_gesture_buffer(){ // 버튼이 눌리고 있을 때만 호출
  
  Serial.print(last_result_before);
  Serial.print(" -> ");
  Serial.print(last_result);

  Serial.print(" : ");
  Serial.print(stResult);
  Serial.print(" - ");
  Serial.println(before_status);
  
  if(last_result_before == last_result){
    if(before_status != last_result){
      stResult = 1;
      before_status = last_result;
    }else{
      stResult = stResult>SIGNATURE_TRAIN_VALUE?SIGNATURE_TRAIN_VALUE:stResult+1;
      
    }
  }else{
    if(stResult >= SIGNATURE_TRAIN_VALUE ){
      if(before_status != UNKNOWN && before_status != SILENT && command_buffer_occupied<COMMAND_BUFFER_SIZE){
        command_buffer[command_buffer_occupied++] = before_status;
        
      }
    }
    before_status = UNKNOWN;
  }

}

uint8_t handle_command_buffer(){ // return command idx. when button released
  //Handle Last Status
  if(stResult >= SIGNATURE_TRAIN_VALUE && before_status != UNKNOWN && before_status != SILENT && command_buffer_occupied<COMMAND_BUFFER_SIZE)command_buffer[command_buffer_occupied++] = before_status;
  //Serial.print("COMMAND: ");

  //for(int i=0;i<command_buffer_occupied;i++){
  //  Serial.print(command_buffer[i]);
  //  Serial.print(" ");
  //}
  //Serial.println();
  if(command_buffer_occupied == 0){
    return 0;
  }

  bool dropped[COMMAND_LIST_SIZE] = {0,};

  for(int i=0;i<command_buffer_occupied;i++){
    for(int j=0;j<COMMAND_LIST_SIZE;j++){
      if(dropped[j])continue;

      if(!command_list[j][0] || !command_list[j][1]){
        dropped[j] = true;
        continue;
      }

      if(command_list[j][1] != command_buffer_occupied){
        dropped[j] = true;
        continue;
      }

      if(command_buffer[i] != command_list[j][2+i]){
        dropped[j] = true;
        continue;
      }
    }
  }

  for(int i=0;i<COMMAND_LIST_SIZE;i++){
    if(!dropped[i]){
      uint8_t packet = create_gesture_packet(i);
      //SE
      if(is_synced){
        notifyCharacteristic.writeValue(&packet, 1);
      }
      break;
    }
  }

  command_buffer_occupied = 0; // 커맨드 스택 클리어
  return 0; // flush gesture buffer
}

void reset_command_list(){
  for(int i=0;i<COMMAND_LIST_SIZE;i++){
    command_list[i][0] = 0; // flush data
    command_list[i][1] = 0;
  }
  command_list_size = 0;
}




uint8_t create_gesture_packet(uint8_t idx){
  uint8_t base_packet = 0b00000000; // base_packet starts with 0.
  uint8_t send_data = idx & 0b00111111; // idx will be 0~31.
  return base_packet | send_data;

  //return idx & 0b00111111;
}

bool create_command_packet(uint8_t idx, uint8_t* packet, uint8_t size = 255){ // UNUSED
  if(!command_list[idx][0])return false; // there is no command;
  if(!command_list[idx][1])return false; // length is 0;

  if(packet==NULL) return false; // null ptr
  uint8_t start_packet_base = 0b01000000;
  uint8_t length = command_list[idx][1] & 0b00111111;

  if(size != 255 && size < length + 2) return false; // buffer is too small

  packet[0] = start_packet_base|(length+2);
  packet[1] = idx;
  for(int i=0;i<length;i++){ // Command len must be more than 1.
    packet[i+2] = command_list[idx][2+i];
  }

  return true;
  
}

uint8_t send_command_hash(){ // 해시 계산 후 전달
  const uint8_t commands_bta = (COMMAND_LIST_SIZE + 7)/8;
  const uint8_t hashlen = 10;
  const uint8_t hv[4] = {HKEY0, HKEY1, HKEY2, HKEY3};

  memset(outgoing_buffer, 0, 20);

  outgoing_buffer[0] = 0b01000000 | (commands_bta + hashlen + 2);//header;
  outgoing_buffer[1] = 0b00000001;
  outgoing_buffer[2] = commands_bta;
  outgoing_buffer[3] = hashlen;

  for(int i=0;i<hashlen;i++){
    outgoing_buffer[4+commands_bta+i] = hv[i%4];
  }

  for(uint8_t i=0;i<COMMAND_LIST_SIZE;i++){
    if(!command_list[i][0])continue;
    if(!command_list[i][1])continue; // ERROR

    uint8_t bt_idx = i/8;
    uint8_t bt_hdx = i%8;

    outgoing_buffer[4+bt_idx] |= (1<<bt_hdx);

    for(uint8_t j=0;j<command_list[i][1];j++){
      uint8_t param_idx = i*COMMAND_BUFFER_SIZE + j;
      uint8_t hash_idx = param_idx%hashlen;
      uint8_t mixval = command_list[i][j+2]^param_idx;
      outgoing_buffer[4+commands_bta+hash_idx] = (outgoing_buffer[4+commands_bta+hash_idx]^mixval)*0x93;
    }
  }


  return 0;
}

uint8_t return_ack_with_seq(uint8_t seq){
  return 0b11000000 | (seq & 0b00111111);
}

uint8_t return_err_with_code(uint8_t code){
  return 0b10000000 | (code & 0b00111111);
}

//const uint8_t ack_packet = 0b11000000;
//const uint8_t err_packet = 0b10000000;

//ack with 1 is Sync.


//PC 데이터 양식

// 0b 00 000000 -> 기본 데이터 컨테이너
// 0b 11 000000 -> ACK
// 0b 10 000000 -> ERR
// 0b 01 000000 -> Reserve

//아두이노 데이터 양식

// 0b 00 xxxxxx -> 제스처 바이트
// 0b 11 xxxxxx -> ACK
// 0b 10 xxxxxx -> ERR
// 0b 01 xxxxxx -> DAT

//initial_handshake

//PC -> ARDUINO ACK 0
//ARDUINO -> PC ACK 0 if not have hash
//ARDUINO -> PC 0b01 data_len 0b00000001 [command hash] if have hash
//PC -> ARDUINO ACK 0 if hash is correct
//PC -> ARDUINO SEND_COMMANDS if hash is not correct(부분 전달 차후 구현)
//ARDUINO READY when PC SEND ACK 1
//ARDUINO SEND HASH when PC Send COMMAND
//PC Will check Arduino if have some hash

//Special Data 

// ACK 111111 -> Heartbeat.
// ERR 111111 -> Pls Resend

uint8_t get_seq_from_packet(uint8_t packet){
  return 0b00111111 & packet;
}

//PC는 Header -> IDX -> Packets 순으로 전달한다.

uint8_t get_gesture_from_pc(uint8_t* packet, uint8_t size = 255){ // PC에서 패킷을 보낼 때는 0b00 xxxxxx을 사용.
  if(!packet)return 255;//err

  

  uint8_t length = get_seq_from_packet(packet[0]);
  if(length < 2) return 255;
  if(length+1 > size) return 255;//buffer is too small
  if(packet[1] >=COMMAND_LIST_SIZE) return 255;
  if(length-2 > COMMAND_BUFFER_SIZE ) return 255;

  for(int i=0;i<length;i++){
    command_list[packet[1]][i+1] = packet[i+1];
  }
  
  command_list[packet[1]][1] = length-2; // 커맨드 길이
  command_list[packet[1]][0] = 1;

  
}




int readRegister(uint8_t slaveAddress, uint8_t Address){ // Arduino nano 33 BLE는 WIRE 라인이 2개 있다. Bluetooth가 Serial1인 것과 비슷.
  #ifdef DEBUG_OUT_FLAG
  Serial.print("Transmission Ready to [");
  Serial.print(slaveAddress);
  Serial.print("] ->");
  #endif
  Wire1.beginTransmission(slaveAddress);
  #ifdef DEBUG_OUT_FLAG
  Serial.print(" Begin Write to [");
  Serial.print(Address);
  Serial.print("] -> ");
  #endif
  Wire1.write(Address);
  #ifdef DEBUG_OUT_FLAG
  Serial.print("Transmission Fin. -> ");
  #endif
  if(Wire1.endTransmission() != 0)return -1;
  #ifdef DEBUG_OUT_FLAG
  Serial.print("Transmission Con -> ");
  #endif
  if(Wire1.requestFrom(slaveAddress,1)!=1) return -1;
  #ifdef DEBUG_OUT_FLAG
  Serial.print("READ Value: -> ");
  #endif
  return Wire1.read();
}

void onSyncWritten(BLEDevice central, BLECharacteristic characteristic){

  int len = characteristic.readValue(incoming_buffer, 20);
  if(len<1) return;

  uint8_t header = incoming_buffer[0];

  if(header == 0xC0){ // ACK 0
    if(have_command){
      send_command_hash();
      syncCharacteristic.writeValue(outgoing_buffer, 20);
    }else{
      uint8_t packet = return_ack_with_seq(0);
      syncCharacteristic.writeValue(&packet, 1);
    }
  }

  else if(header == 0b00000001){

    if(incoming_buffer[1] == 0b00000010){
      send_command_hash();
      syncCharacteristic.writeValue(outgoing_buffer, 20);
    }

    else if(incoming_buffer[1] == 0b00000011){
      reset_command_list();
      uint8_t packet = return_ack_with_seq(3);
      syncCharacteristic.writeValue(&packet, 1);
    }
    
  }

  else if(header == 0xC1){ // ACK 1
    is_synced = true;
    have_command = true;
  }

  else if((header&0xC0)==0x00){
    uint8_t err = get_gesture_from_pc(incoming_buffer, 20);
  }

  else if((header&0xC0) == 0x80){
    is_synced = false; //오류 시 연결 끊기
  }

}


void setup() {

  const int kInferencesPerCycle = 1000;

  while(!BLE.begin()){};
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(gestureService);

  syncCharacteristic.setEventHandler(BLEWritten, onSyncWritten);

  gestureService.addCharacteristic(notifyCharacteristic);
  gestureService.addCharacteristic(syncCharacteristic);
  BLE.addService(gestureService);

  

  
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  Serial.print("TFLite Version is: ");
  Serial.println(TFLITE_VERSION_STRING);
  Serial.print("TFLite Schema Version is: ");
  Serial.println(TFLITE_SCHEMA_VERSION);

  if(!IMU.begin()){
    TF_LITE_REPORT_ERROR(error_reporter, "Failed to initialize IMU.");
    return;
  }


  float sample_rate = IMU.accelerationSampleRate();
  float sample_rate_rot = IMU.gyroscopeSampleRate();

  pinMode(GESTURE_BUTTON, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Serial.print("ACCEL SAMPLE RATE: ");
  Serial.println(sample_rate);
  Serial.print("GYRO SAMPLE RATE: ");
  Serial.println(sample_rate_rot);

  model = tflite::GetModel(quantized_model_int8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  //static tflite::ops::micro::AllOpsResolver resolver; // MEAN 없음
  static tflite::MicroOpResolver<30> resolver; // 빌트인 함수는 복제 메커니즘 상 Resolver 슬롯을 버전마다 하나씩 차지한다. 넉넉하게 넣어두기.

  resolver.AddBuiltin(
    tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
    tflite::ops::micro::Register_DEPTHWISE_CONV_2D(),1,3);
  
  resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                      tflite::ops::micro::Register_CONV_2D(),1,3);
  
  resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
           tflite::ops::micro::Register_FULLY_CONNECTED(),1,4);

  resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
         tflite::ops::micro::Register_SOFTMAX(),1,3);
   resolver.AddBuiltin(tflite::BuiltinOperator_RELU,
                               tflite::ops::micro::Register_RELU(),1,3);
  //resolver.AddBuiltin(tflite::BuiltinOperator_MEAN,
  //                             tflite::ops::micro::Register_MEAN(),1,3);     

  resolver.AddBuiltin(tflite::BuiltinOperator_AVERAGE_POOL_2D,
                               tflite::ops::micro::Register_AVERAGE_POOL_2D(),1,2);   

  resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                      tflite::ops::micro::Register_RESHAPE(),1,1) ;      


  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if(allocate_status != kTfLiteOk){
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }



  input = interpreter->input(0);
  output = interpreter->output(0);

  inference_count = 0;


  //BLE 연결이 끝나면 여기서부터 시작한다.
  //READY 

  uint8_t init_val = 0;
  notifyCharacteristic.writeValue(init_val);

  send_command_hash();
  syncCharacteristic.writeValue(outgoing_buffer, 20);


  BLE.advertise();
  //여기서 연결 확인 후 체크를 시작한다

  digitalWrite(LED_RED, LOW);    
  digitalWrite(LED_GREEN, HIGH);

  while(!is_synced){
    BLE.poll();//우선 이벤트를 처리한다.
    delay(5);
  }

  digitalWrite(LED_RED, HIGH);    
  digitalWrite(LED_GREEN, LOW); // 기계 준비됨
  IMU.setContinuousMode();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  #ifdef DEBUG_OUT_FLAG
  //uint8_t random_data[6]; //useless
  //Serial.print("Blocked? -> ");
  //Serial.println(readRegister(0x6b, 0x2F) & 63); // 라이브러리가 제대로 된 수를 리턴 안하니 라이브러리에 있는 함수에서 긁어오자.
  
  uint32_t tstart = micros();
  #endif

  if((readRegister(0x6b, 0x2F) & 63) < DATA_QUEUE_SIZE)return;
  static int cnt = 1;
  while(IMU.accelerationAvailable()){
    if(cnt!=DATA_SAMPLE_COUNT) {
      IMU.readAcceleration(imu_data_buffer[0],imu_data_buffer[1],imu_data_buffer[2]);
      IMU.readGyroscope(imu_data_buffer[3],imu_data_buffer[4],imu_data_buffer[5]);
      cnt++;
      continue;
      }
    int readpts = queue_last_pts*IMU_SAMPLE_CHANNEL;
    IMU.readAcceleration(imu_data_queue[readpts + 0],imu_data_queue[readpts + 1],imu_data_queue[readpts + 2]);
    IMU.readGyroscope(imu_data_queue[readpts + 3],imu_data_queue[readpts + 4],imu_data_queue[readpts + 5]);
    for(int i=0;i<6;i++){
      baselines[i] = (BASELINE_ALPHAS[i] * baselines[i]) + (1.0-BASELINE_ALPHAS[i])*imu_data_queue[readpts +i]; // update baselines
      time_baselines[readpts + i] = baselines[i];
    }
    queue_last_pts++;
    cnt=1;
    if(!queue_is_full && queue_last_pts == IMU_SAMPLE_COUNT)queue_is_full=1;
    if(queue_last_pts == IMU_SAMPLE_COUNT)queue_last_pts = 0;
  }

  #ifdef DEBUG_OUT_FLAG
  Serial.print("QUEUE_PTS: ");
  Serial.println(queue_last_pts);
  #endif
  for(int i=0;i<30*6;i++){ // 데이터 뭐시기 루프
    int readpts = ((queue_last_pts)*6+i)%(30*6);
    float raw = imu_data_queue[readpts];
    float delta = raw - time_baselines[readpts];
    delta *= MULT_ARRAY[i%6];

    #ifdef USE_OFFSET
    delta -= MULT_OFFSET[i%6];
    #endif

    if (delta > 127.0f) delta = 127.0f;
    if (delta < -128.0f) delta = -128.0f; // 클램핑

    input->data.int8[i] = (int8_t) delta;
  }
  #ifdef DEBUG_OUT_FLAG
  uint32_t start = micros();
  #endif
  
  if(interpreter->Invoke() != kTfLiteOk)return;
  #ifdef DEBUG_OUT_FLAG
  itime = micros()-start;
  ttime = micros() - tstart;
  #endif

  handle_result(interpreter->output(0)->data.int8);

  if(!digitalRead(GESTURE_BUTTON)){
    handle_gesture_buffer();
    if(!before_button_status)before_button_status = true;
  }
  
  if(before_button_status && digitalRead(GESTURE_BUTTON)){
    handle_command_buffer();
    before_button_status = false;
  }

  //delay(300);
  #ifdef DEBUG_OUT_FLAG
  Serial.print("Inference Count - ");
  Serial.println(inference_count);
  inference_count++;
  Serial.print("T<us>: ");
  Serial.println(itime);
  Serial.print("Total T<us>: ");
  Serial.println(ttime);
  #endif

  if(!is_synced || !BLE.central().connected()){
      digitalWrite(LED_RED, LOW);    
      digitalWrite(LED_GREEN, HIGH);

    IMU.setOneShotMode();
    is_synced = false;
    while(!is_synced){
      BLE.poll();
      delay(5);
    }
    //큐 비우기
    queue_last_pts = 0; 
    queue_is_full = false;

    IMU.setContinuousMode();

    digitalWrite(LED_RED, HIGH);    
    digitalWrite(LED_GREEN, LOW); // 기계 준비됨
  }
  BLE.poll();
}
