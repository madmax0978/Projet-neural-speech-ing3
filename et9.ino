#include <arduinoMFCC.h>
#include <NeuralNetwork.h>
#include "parametre.h"


// === CONFIGURATION ===
#define ADC_PIN A0
#define BUTTON_PIN 22
#define RECORD_SIZE 8000
#define FRAME_SIZE 256
#define HOP_SIZE 128
#define NUM_FRAMES ((RECORD_SIZE - FRAME_SIZE) / HOP_SIZE + 1)
#define MFCC_SIZE 26
#define DCT_SIZE 13
#define LED1 23
#define LED2 24

// === BUFFERS ===
uint16_t recordBuffer[RECORD_SIZE];
uint16_t frames[NUM_FRAMES][FRAME_SIZE];
float mfccMatrix[NUM_FRAMES][DCT_SIZE];

// === IA ===
#define INPUT_SIZE 793
#define OUTPUT_SIZE 2
const unsigned int layers[] = {793, 64, 32, 2};
NeuralNetwork NN(layers, (float*)weights_data, (float*)biases_data, 4);

// === FILTRE ===
#define RIF_TAPS 53
int16_t firCoeffs[RIF_TAPS] = { -248, 129, 234, 276, 170, -56, -259, -277, -56, 274,
  462, 318, -121, -569, -658, -222, 519, 1038, 838, -150,
  -1387, -1928, -964, 1632, 5091, 8043, 9202, 8043, 5091, 1632,
  -964, -1928, -1387, -150, 838, 1038, 519, -222, -658, -569,
  -121, 318, 462, 274, -56, -277, -259, -56, 170, 276,
  234, 129, -248 };
int16_t firBuffer[RIF_TAPS] = {0};
int firIndex = 0;

volatile uint16_t adcRaw = 0;
volatile bool sampleReady = false;
bool recording = false;
int recordedSamples = 0;
int downsampleCounter = 0;

arduinoMFCC mfcc(MFCC_SIZE, DCT_SIZE, FRAME_SIZE, 8000.0);

uint16_t applyFIR(uint16_t input) {
  firBuffer[firIndex] = input;
  int32_t acc = 0;
  int idx = firIndex;
  for (int i = 0; i < RIF_TAPS; i++) {
    acc += (int32_t)firCoeffs[i] * firBuffer[idx];
    idx = (idx == 0) ? RIF_TAPS - 1 : idx - 1;
  }
  firIndex = (firIndex + 1) % RIF_TAPS;
  int32_t filtered = acc >> 15;
  if (filtered < 0) filtered = 0;
  if (filtered > 4095) filtered = 4095;
  return (uint16_t)filtered;
}

void extractFrames() {
  for (int i = 0; i < NUM_FRAMES; i++) {
    int start = i * HOP_SIZE;
    for (int j = 0; j < FRAME_SIZE; j++) {
      frames[i][j] = (start + j < RECORD_SIZE) ? recordBuffer[start + j] : 0;
    }
  }
}

void computeMFCCs() {
  for (int i = 0; i < NUM_FRAMES; i++) {
    float inputFrame[FRAME_SIZE];
    for (int j = 0; j < FRAME_SIZE; j++) {
      inputFrame[j] = (float)frames[i][j];
    }
    float mfccResult[DCT_SIZE];
    mfcc.computeWithDCT(inputFrame, mfccResult);
    for (int k = 0; k < DCT_SIZE; k++) {
      mfccMatrix[i][k] = mfccResult[k];
    }
  }
}

void normalizeMFCCs() {
  for (int col = 0; col < DCT_SIZE; col++) {
    float sum = 0, variance = 0;
    for (int row = 0; row < NUM_FRAMES; row++) sum += mfccMatrix[row][col];
    float mean = sum / NUM_FRAMES;
    for (int row = 0; row < NUM_FRAMES; row++)
      variance += pow(mfccMatrix[row][col] - mean, 2);
    float stddev = sqrt(variance / NUM_FRAMES);
    for (int row = 0; row < NUM_FRAMES; row++)
      mfccMatrix[row][col] = (mfccMatrix[row][col] - mean) / stddev;
  }
}

void flattenMFCCs(float output[793]) {
  int idx = 0;
  for (int i = 0; i < NUM_FRAMES; i++)
    for (int j = 0; j < DCT_SIZE; j++)
      output[idx++] = mfccMatrix[i][j];
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // ADC 32 kHz via timer
  analogReadResolution(12);
  ADC->ADC_MR &= ~ADC_MR_LOWRES;
  ADC->ADC_CHER = ADC_CHER_CH7;
  ADC->ADC_CR = ADC_CR_START;
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
  TC0->TC_CHANNEL[0].TC_RC = 42000000 / 32000;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  NVIC_EnableIRQ(TC0_IRQn);

  // MFCC
  mfcc.create_hamming_window();
  mfcc.create_mel_filter_bank();
  mfcc.create_dct_matrix();

}

// === LOOP ===
void loop() {
  if (!recording && digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == HIGH) {
      recordedSamples = 0;
      recording = true;
      Serial.println("== DEBUT ENREGISTREMENT ==");
    }
  }

  if (recording && sampleReady) {
    sampleReady = false;
    if (++downsampleCounter == 4) {
      downsampleCounter = 0;
      uint16_t filtered = applyFIR(adcRaw);
      recordBuffer[recordedSamples++] = filtered;
      if (recordedSamples >= RECORD_SIZE) {
        recording = false;
        Serial.println("== FIN ENREGISTREMENT ==");
        extractFrames();
        computeMFCCs();
        normalizeMFCCs();
        
        float inputVector[793];
        flattenMFCCs(inputVector);
        float *prediction = NN.FeedForward(inputVector);

        Serial.print("Estimation Jaune: "); Serial.print(prediction[0], 4);
        Serial.print(" / Vert: "); Serial.println(prediction[1], 4);

        if (abs(prediction[0] - prediction[1]) < 0.1) {
          Serial.println("Incertitude : allumage neutre");
          digitalWrite(LED1, LOW);
          digitalWrite(LED2, LOW);
        }
        else if (prediction[0] > prediction[1]) {
          digitalWrite(LED1, HIGH);
          digitalWrite(LED2, LOW);
        } else {
          digitalWrite(LED1, LOW);
          digitalWrite(LED2, HIGH);
}
      }
    }
  }
}

// ISR ADC
void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;
  adcRaw = ADC->ADC_CDR[7];
  sampleReady = true;
  ADC->ADC_CR = ADC_CR_START;
}
