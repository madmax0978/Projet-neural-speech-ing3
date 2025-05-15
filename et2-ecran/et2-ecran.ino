#include <Arduino.h>
#include <arduinoFFT.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// === PARAMÈTRES FFT / OLED ===
#define BUFFER_SIZE 128
#define SAMPLING_FREQ 32000.0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, BUFFER_SIZE, SAMPLING_FREQ);

// === FILTRE RIF FIR ===
#define RIF_TAPS 53
int16_t firCoeffs[RIF_TAPS] = {
  -248, 129, 234, 276, 170, -56, -259, -277, -56, 274,
  462, 318, -121, -569, -658, -222, 519, 1038, 838, -150,
  -1387, -1928, -964, 1632, 5091, 8043, 9202, 8043, 5091, 1632,
  -964, -1928, -1387, -150, 838, 1038, 519, -222, -658, -569,
  -121, 318, 462, 274, -56, -277, -259, -56, 170, 276,
  234, 129, -248
};
int16_t firBuffer[RIF_TAPS] = {0};
int firIndex = 0;

volatile uint16_t adcRaw = 0;
volatile bool sampleReady = false;


uint16_t applyFIR(uint16_t input) {
  firBuffer[firIndex] = input;

  int32_t acc = 0;
  int idx = firIndex;
  for (int i = 0; i < RIF_TAPS; i++) {
    acc += (int32_t)firCoeffs[i] * firBuffer[idx];
    idx = (idx == 0) ? RIF_TAPS - 1 : idx - 1;
  }

  firIndex = (firIndex + 1) % RIF_TAPS;

  // Normalisation pour ramener à une valeur 12 bits (0 - 4095)
  int32_t filtered = acc >> 15; // ajustable selon le gain total
  if (filtered < 0) filtered = 0;
  if (filtered > 4095) filtered = 4095;

  return (uint16_t)filtered;
}


void setupADC() {
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
}

void setupOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 failed"));
    while (true) {}
  }
  display.clearDisplay();
  display.display();
}

int barLength(double d) {
  float fy = 10.0 * (log10(d + 1) + 1.1);
  int y = constrain((int)fy - 30, 0, 64);
  return y;
}

void showSpectrum() {
  int displayFFTvalue[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; i++) displayFFTvalue[i] = barLength(vReal[i]);
  for (int i = 0; i < 64; i++) {
    display.drawLine(i * 2, 64, i * 2, 64 - displayFFTvalue[i], WHITE);
  }
}

void fillBuffer() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    while (!sampleReady) {}
    noInterrupts();
    uint16_t filtered = applyFIR(adcRaw);
    sampleReady = false;
    interrupts();
    vReal[i] = filtered;
    vImag[i] = 0;
  }
}

void setup() {
  Serial.begin(115200);
  setupOLED();
  setupADC();
}

void loop() {
  fillBuffer();
  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, BUFFER_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, BUFFER_SIZE);

  display.clearDisplay();
  showSpectrum();
  display.display();
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;
  adcRaw = ADC->ADC_CDR[7];
  sampleReady = true;
  ADC->ADC_CR = ADC_CR_START;
}
