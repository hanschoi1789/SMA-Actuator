#include <Encoder.h>
#include <HX711.h>

#define DOUT 10
#define CLK 9
#define calibration_factor -25900

HX711 scale;
Encoder myEnc(2, 3);

const float resolution_mm = 0.04;

void setup() {

  Serial.begin(115200); 
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.tare();
  
  // 시작 시 확인 메시지
  //Serial.println("Data Start (80Hz Mode)");
}

void loop() {
  // 1. 엔코더는 항상 최신 값을 유지 (인터럽트 방식)
  long pos = myEnc.read();
  float displacement = pos * resolution_mm;

  // 2. HX711 데이터가 준비되었을 때만 (약 12.5ms 마다) 출력
  if (scale.is_ready()) {
    float force = scale.get_units(1); // 80Hz 모드이므로 1회만 읽음
    // 3. 출력 형식 최적화 (단위 명칭 포함)
    
    Serial.print(displacement, 3); 
    Serial.print(", ");
   
    Serial.println(force, 3); 
   
  }
  
  // 별도의 delay가 없어도 scale.is_ready()가 속도를 제어합니다.
}