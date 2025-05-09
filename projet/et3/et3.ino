#define RIF_TAPS 53
#define RECORD_SIZE 8000  // 10 cycles de 800 échantillons
#define SAMPLE_DIV 4      // Pour passer de 32 kHz à 8 kHz

volatile uint16_t adcRaw = 0;
volatile bool sampleReady = false;

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

uint16_t recordBuffer[RECORD_SIZE];
int recordedSamples = 0;
int downsampleCounter = 0;

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
  int32_t filtered = acc >> 14; // ajustable selon le gain total
  if (filtered < 0) filtered = 0;
  if (filtered > 4095) filtered = 4095;

  return (uint16_t)filtered;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  ADC->ADC_MR &= ~ADC_MR_LOWRES;
  ADC->ADC_CHER = ADC_CHER_CH7;
  ADC->ADC_CR = ADC_CR_START;

  // Timer 32 kHz
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
  TC0->TC_CHANNEL[0].TC_RC = 42000000 / 32000;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  NVIC_EnableIRQ(TC0_IRQn);

  Serial.println("== DEBUT ACQUISITION ET3 ==");
}

void loop() {
  if (sampleReady && recordedSamples < RECORD_SIZE) {
    sampleReady = false;

    if (++downsampleCounter == SAMPLE_DIV) {
      downsampleCounter = 0;

      unsigned long t1 = micros();
      uint16_t filtered = applyFIR(adcRaw);
      unsigned long t2 = micros();

      Serial.print("Temps filtrage (µs) : ");
      Serial.println(t2 - t1);

      if (filtered < 0) filtered = 0;
      if (filtered > 4095) filtered = 4095;

      recordBuffer[recordedSamples++] = (uint16_t)filtered;


      if (recordedSamples == RECORD_SIZE) {
        Serial.println("== FIN VERIFICATION ==");
      }
    }
  }
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;
  adcRaw = ADC->ADC_CDR[7];
  sampleReady = true;
  ADC->ADC_CR = ADC_CR_START;
}
