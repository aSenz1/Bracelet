#include <Arduino.h>
#include <Wire.h>

// Pinos
const int hallPin = 2;       // Pino do sensor Hall (interrupção)
const int button = 3;        // Pino do botão
const int ida = 5;          // Pino PWM sentido 1
const int volta = 6;       // Pino PWM sentido 2
// Variáveis de estado
volatile bool direcaoIda = false;
bool controleAtivo = false;  // Indica se o controle automático está ativo
float desiredFreq = 0.0;     // Frequência desejada (setada via Serial)
float currentPWM = 0;        // Valor atual do PWM (0-255)
const float Kp = 100;        // Ganho proporcional (ajuste conforme necessário)
const float Kd = 50;        // Ganho derivativo (ajuste conforme necessário)
const float Ki = 0.01;        // Ganho integral (ajuste conforme necessário)
// Variáveis para medição de frequência
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
float frequency = 0.0;
float error = 0.0;          // Erro entre frequência desejada e medida
float preverror = 0.0;      // Erro anterior (para controle PID)
float integral = 0.0;        // Integral do erro (para controle PID)
// Médias
float lastFrequency = 0;
float avg2 = 0;
int count2 = 0;

// Média móvel de 10 leituras
const int WINDOW_SIZE = 10;
float readings[WINDOW_SIZE];
float sum10 = 0;
int index10 = 0;
float avg10 = 0;
bool avg10_ready = false;

void buttonPress() {
  Serial.print("APERTOU O BOTAO");
  direcaoIda = !direcaoIda;
  
  if (direcaoIda) {
    digitalWrite(ida, HIGH);
    digitalWrite(volta, LOW);
  } else {
    digitalWrite(ida, LOW);
    digitalWrite(volta, LOW);
  }
}

void sendToPlotter() {
  Serial.print("Freq"); Serial.print(","); Serial.print(frequency,3);
  /*Serial.print(",Avg2"); Serial.print(","); Serial.print(avg2);
  if (avg10_ready) {
    Serial.print(",Avg10"); Serial.print(","); Serial.print(avg10);
  } else {
    Serial.print(",Avg10,"); Serial.print("0");
  }
  */
  Serial.print(",Desired"); Serial.print(","); Serial.print(desiredFreq);
  Serial.print(",PWM"); Serial.print(","); Serial.print(currentPWM);
  
  Serial.println();
}

void updatePWMControl() {
  if (!controleAtivo || desiredFreq <= 0) return;

  static unsigned long lastControlTime = 0;
  unsigned long now = millis();
  
  // Calcula dt em segundos (com proteção contra overflow)
  float dt = (now - lastControlTime) / 1000.0;
  if (dt <= 0) { // Se dt inválido, usa 0.01s (10ms)
    dt = 0.01;
  }
  
  // Usa a média de 2 leituras
  float currentFreq = avg2;
  
  // Proteção contra divisão por zero
  if (currentFreq <= 0) {
    currentFreq = 0.001; // Valor mínimo para evitar NaN
  }

  // Cálculo do erro
  float newError = desiredFreq - currentFreq;
  
  // Termo Proporcional
  float P = Kp * newError;
  
  // Termo Integral com anti-windup
  integral += Ki * newError * dt;
  integral = constrain(integral, -255, 255);
  
  // Termo Derivativo (protegido contra valores extremos)
  float derivative = 0;
  if (dt > 0.001) { // Só calcula se dt for significativo
    derivative = (newError - preverror) / dt;
    derivative = constrain(derivative, -100, 100); // Limita a variação
  }
  float D = Kd * derivative;
  
  // Calcula o novo PWM
  float newPWM = currentPWM + P + integral + D;
  
  // Limites de segurança
  newPWM = constrain(newPWM, 0, 255);
  
  
  // Atualiza variáveis
  currentPWM = newPWM;
  preverror = newError;
  lastControlTime = now;
}

void detectPulse() {
  unsigned long currentTime = micros();
  unsigned long interval = currentTime - lastPulseTime;
  int rawValue = analogRead(A0);
  if (interval > 5000 && rawValue < 30) { // Filtro de debounce (5ms)
    if (lastPulseTime != 0) {
      pulseInterval = interval;
      frequency = 1000000.0 / interval; // Frequência em Hz
      
      error = desiredFreq - frequency;
      preverror = error; // Armazena o erro atual para o controle PID
      // Média a cada 2 leituras
      lastFrequency = frequency;
      count2++;
      
      if (count2 == 2) {
        avg2 = (lastFrequency + frequency) / 2.0;
        count2 = 0;
      }
      /*
      // Média móvel de 10 leituras
      sum10 -= readings[index10];
      readings[index10] = frequency;
      sum10 += readings[index10];
      index10 = (index10 + 1) % WINDOW_SIZE;
      
      if (index10 == 0 && !avg10_ready) {
        avg10_ready = true;
      }
      
      if (avg10_ready) {
        avg10 = sum10 / WINDOW_SIZE;
      }*/
      if (!controleAtivo || desiredFreq <= 0) return;

      static unsigned long lastControlTime = 0;
      unsigned long now = millis();
      
      // Calcula dt em segundos (com proteção contra overflow)
      float dt = (now - lastControlTime) / 1000.0;
      if (dt <= 0) { // Se dt inválido, usa 0.01s (10ms)
        dt = 0.01;
      }
      
      // Usa a média de 2 leituras
      float currentFreq = avg2;
      
      // Proteção contra divisão por zero
      if (currentFreq <= 0) {
        currentFreq = 0.001; // Valor mínimo para evitar NaN
      }

      // Cálculo do erro
      float newError = desiredFreq - currentFreq;
      
      // Termo Proporcional
      float P = Kp * newError;
      
      // Termo Integral com anti-windup
      integral += Ki * newError * dt;
      integral = constrain(integral, -255, 255);
      
      // Termo Derivativo (protegido contra valores extremos)
      float derivative = 0;
      if (dt > 0.001) { // Só calcula se dt for significativo
        derivative = (newError - preverror) / dt;
        derivative = constrain(derivative, -100, 100); // Limita a variação
      }
      float D = Kd * derivative;
      
      // Calcula o novo PWM
      float newPWM = currentPWM + P + integral + D;
      
      // Limites de segurança
      newPWM = constrain(newPWM, 0, 255);
      
      // Aplica gradualmente mudanças bruscas
      float maxChange = 20.0; // Máxima variação por ciclo
      if (abs(newPWM - currentPWM) > maxChange) {
        newPWM = currentPWM + (newPWM > currentPWM ? maxChange : -maxChange);
      }
      
      // Atualiza variáveis
      currentPWM = newPWM;
      preverror = newError;
      lastControlTime = now;
    }
    lastPulseTime = currentTime;
  }

}

void ligdes() {
  int estado_botao = digitalRead(button);

  if(estado_botao == HIGH) {
    Serial.print("BOTAO PRESSIONADO----------");    
    direcaoIda = !direcaoIda;
    
    if (direcaoIda) {
      Serial.println("LIGA");
      if (!controleAtivo) {
        analogWrite(ida, 255);
        analogWrite(volta, 0);
      }
    } else {
      Serial.println("DESLIGA");
      controleAtivo = false;
      digitalWrite(ida, LOW);
      digitalWrite(volta, LOW);
    }
    
    while(digitalRead(button) == HIGH) {
      delay(10);
    }
  }
}

void checkHallSensor() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) {
    int rawValue = analogRead(A0); // Conecte o sensor a A0 também
    Serial.print("Digital: ");
    Serial.print(digitalRead(hallPin));
    Serial.print(" | Analog: ");
    Serial.println(rawValue);
    lastPrint = millis();
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("FREQ ")) {
      desiredFreq = command.substring(5).toFloat();
      controleAtivo = (desiredFreq > 0);
      currentPWM = 170; // Valor inicial do PWM
      
      Serial.print("Frequência desejada: ");
      Serial.print(desiredFreq);
      Serial.println(" Hz");
      Serial.print("Controle ");
      Serial.println(controleAtivo ? "ATIVADO" : "DESATIVADO");
      
      if (controleAtivo) {
        direcaoIda = true;
        digitalWrite(volta, LOW);
      }
    }
    else if (command.equals("STOP")) {
      controleAtivo = false;
      desiredFreq = 0;
      digitalWrite(ida, LOW);
      digitalWrite(volta, LOW);
      Serial.println("Controle DESATIVADO");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ida, OUTPUT);
  pinMode(volta, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), detectPulse, FALLING);

  digitalWrite(ida, LOW);
  digitalWrite(volta, LOW);
  
  // Inicializa o array de médias
  for (int i = 0; i < WINDOW_SIZE; i++) {
    readings[i] = 0;
  }
  
  Serial.println("Sistema pronto. Envie:");
  Serial.println("FREQ <valor> - Para setar a frequência desejada");
  Serial.println("STOP - Para desativar o controle");
}

void loop() {
  // Atualiza o controle PWM se estiver ativo
  //checkHallSensor();
  // Processa comandos da serial
  processSerialCommands();
  analogWrite(ida, (int)currentPWM);
  analogWrite(volta, 0);
  // Envia dados para o Serial Plotter
  if (millis() % 1000 == 0) { // A cada 100ms
    sendToPlotter();
  }
  // Verifica botão manual
  //ligdes();
}

/*#include <Wire.h>
#include <math.h>  // Adicionado para funções matemáticas

#define ida 5
#define volta 6
const int button = 3;

// Constante e variáveis relativas ao mpu
const int MPU_ADDR = 0x68;
float acc_x, acc_y, acc_z;
const float ACCEL_SCALE = 16384.0; // para ±2g
const float GRAVITY = 9.81; // m/s²
float acc_offset_x = 0, acc_offset_y = 0, acc_offset_z = 0;

// Variáveis para cálculo da amplitude
float amplitude = 0;
unsigned long startTime = 0;
const float f = 0.6; //Em Hz
const float T = 1/f; //Em segundos

volatile bool direcaoIda = false;
int count_acc_lidas = 0;
const int hallPin = 2;      // Pino do sensor Hall
unsigned long lastTime = 0; // Tempo do último pulso

// Variáveis para frequência instantânea
float frequency = 0;

// Média a cada 2 leituras
float lastFrequency = 0;
float avg2 = 0;
int count2 = 0, count_med = 0;
float prev_value = 0, acc_max = 0, amp_med, som_amp;

// Média a cada 10 leituras
const int WINDOW_SIZE = 10;
float readings[WINDOW_SIZE];
float sum10 = 0;
int index10 = 0;
float avg10 = 0;
bool avg10_ready = false;
bool calculate = false;

void buttonPress() {
  Serial.print("APERTOU O BOTAO");
  direcaoIda = !direcaoIda;
  if (direcaoIda) {
    digitalWrite(ida, HIGH);
    digitalWrite(volta, LOW);
  } else {
    digitalWrite(ida, LOW);
    digitalWrite(volta, LOW);
  }
}

void sendToPlotter() {
  Serial.print("Freq,"); Serial.print(frequency);
  Serial.print(" , freq_avg2,"); Serial.print(avg2);
  
  if (avg10_ready) {
    Serial.print(" Avg10,"); Serial.println(avg10);
  } else {
    Serial.println(",Avg10,0");
  }
  
  // Adiciona a amplitude aos dados plotados
  //Serial.print(",Amplitude,"); Serial.println(amplitude);
}

void detectPulse() {
  unsigned long currentTime = millis();
  unsigned long interval = currentTime - lastTime;
  
  if (interval > 5) { // Filtro de debounce
    if (lastTime != 0) {
      frequency = 1000.0 / interval; // Frequência em Hz

      // --- Média a cada 2 leituras ---
      lastFrequency = frequency;
      count2++;
      if (count2 == 2) {
        avg2 = (lastFrequency + frequency) / 2.0;
        count2 = 0;
      }

      // --- Média móvel de 10 leituras ---
      sum10 -= readings[index10];
      readings[index10] = frequency;
      sum10 += readings[index10];
      index10 = (index10 + 1) % WINDOW_SIZE;

      if (index10 == 0 && !avg10_ready) {
        avg10_ready = true;
      }

      if (avg10_ready) {
        avg10 = sum10 / WINDOW_SIZE;
      }

      sendToPlotter();
    }
    lastTime = currentTime;
  }
}

void readAccelerometer() {
  // Ler os dados brutos do acelerômetro (6 bytes)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
}

void calibrateAccelerometer() {
  Serial.println("Calibrando acelerômetro... Mantenha o sensor nivelado e parado");
  
  float sum_x = 0, sum_y = 0, sum_z = 0;
  const int samples = 2000;
  
  for (int i = 0; i < samples; i++) {
    readAccelerometer();
    sum_x += acc_x;
    sum_y += acc_y;
    sum_z += acc_z;
    delay(10);
  }
  
  acc_offset_x = sum_x / samples;
  acc_offset_y = sum_y / samples;
  acc_offset_z = (sum_z / samples) - ACCEL_SCALE; // Corrigir 1g no eixo Z
  
 Serial.println("Calibração concluída!");
}

void liga_motor(){
  digitalWrite(ida, HIGH);
  digitalWrite(volta, LOW);  
}

void setup() {
  Serial.begin(115200);
  pinMode(ida, OUTPUT);
  pinMode(volta, OUTPUT);
  pinMode(hallPin, INPUT);
  pinMode(button, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(hallPin), detectPulse, FALLING);
  
  digitalWrite(ida, LOW);
  digitalWrite(volta, LOW);
  
  for (int i = 0; i < WINDOW_SIZE; i++) {
    readings[i] = 0;
  }

  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  calibrateAccelerometer();
  
  startTime = millis(); // Inicializa o tempo para cálculo do cosseno
  liga_motor();
}

void loop() {
  
   //Visualizar acy no plotter serial
  /*
  readAccelerometer();
  acc_y = (acc_y - acc_offset_y) * GRAVITY / ACCEL_SCALE;
  Serial.print("Acy"); Serial.print(",") ; Serial.println(acc_y);
  */

    
  // Cálculo do tempo decorrido em segundos
  /* Forma 1 - calculo amplitude
  float currentTime2 = (millis() - startTime) / 1000.0;

  // Verifica se currentTime2 está próximo de 10*T e caso seja, calcula a amplitude

  if (abs(currentTime2 - 10*T)<0.01 && !calculate) { //Margem de erro de 5% e variável para considerar o primeiro cálculo   
     readAccelerometer();
     acc_y = (acc_y - acc_offset_y) * GRAVITY / ACCEL_SCALE;
     amplitude = (abs(acc_y) / pow(2 * PI * f, 2)) * 100; //Amplitude em cm
     Serial.print("Amplitude "); Serial.print(amplitude); Serial.println("cm");
     calculate = true;
     //Serial.print("Acy"); Serial.print(",") ; Serial.println(acc_y);
     // Calcula a amplitude usando a fórmula A = a/(2*pi*f)^2
     //amplitude = (abs(acc_y) / pow(2 * PI * avg10, 2)) * 100; //Amplitude em cm
     //Serial.print("Amplitude "); Serial.print(amplitude); Serial.println("cm");
  }

  *//*
  //Forma 2 de calcular amplitude - armazenando aceleração máxima

  
  readAccelerometer();
  acc_y = (acc_y - acc_offset_y) * GRAVITY / ACCEL_SCALE;
  acc_max = max(abs(prev_value),abs(acc_y));
  prev_value = acc_y;

  count_acc_lidas++;

  //Calculo Amplitude
  if(count_acc_lidas == 100){
     amplitude = (acc_max / pow(2 * PI * f, 2)) * 100; //Amplitude em cm
     Serial.print("Amplitude "); Serial.print(amplitude); Serial.println("cm");        
     count_acc_lidas = 0;
     count_med++;
     som_amp += amplitude; 
  }


  //MÉDIA DE 10 AMPLITUDES
  if(count_med == 10){
    amp_med = som_amp/10;
    Serial.print("Media Amplitude "); Serial.print(amp_med); Serial.println("cm");     
    count_med = 0;
  }

  /*
  int estado_botao = digitalRead(button);
  if (estado_botao == HIGH) {
    Serial.println("BOTAO PRESSIONADO");    
    direcaoIda = !direcaoIda;
    if (direcaoIda) {
      Serial.println("LIGA");
      digitalWrite(ida, HIGH);
      digitalWrite(volta, LOW);
    } else {
      Serial.println("DESLIGA");
      digitalWrite(ida, LOW);
      digitalWrite(volta, LOW);
    }
    delay(300); // Debounce manual do botão
  }
  *//*
  delay(50);
}
*/
