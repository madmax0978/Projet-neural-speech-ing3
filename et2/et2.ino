#define ADC_PIN A0
#define BUFFER_SIZE 1024
#define RIF_TAPS 21

volatile uint16_t buffer[BUFFER_SIZE];
volatile uint16_t indexBuffer = 0;

// Coefficients du filtre FIR pour Fc ≈ 4 kHz à 32 kHz
float firCoeffs[RIF_TAPS] = {
  0.002536, 0.002553, -0.000000, -0.008635, -0.021022, -0.024211, 0.000000,
  0.060557, 0.144592, 0.219131, 0.248999, 0.219131, 0.144592, 0.060557,
  0.000000, -0.024211, -0.021022, -0.008635, -0.000000, 0.002553, 0.002536
};

float firBuffer[RIF_TAPS] = {0};

// Filtrage FIR
float applyFIR(float input) {
  for (int i = RIF_TAPS - 1; i > 0; i--) {
    firBuffer[i] = firBuffer[i - 1];
  }
  firBuffer[0] = input;

  float output = 0;
  for (int i = 0; i < RIF_TAPS; i++) {
    output += firBuffer[i] * firCoeffs[i];
  }
  return output;
}

void setup() {
  Serial.begin(115200);

  // Config ADC
  analogReadResolution(12);
  ADC->ADC_MR &= ~ADC_MR_LOWRES;
  ADC->ADC_CHER = ADC_CHER_CH7;
  ADC->ADC_CR = ADC_CR_START;

  // Config Timer à 32 kHz
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
  TC0->TC_CHANNEL[0].TC_RC = 42000000 / 32000; // 32 kHz
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  NVIC_EnableIRQ(TC0_IRQn);
}

void loop() {
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.println("Valeurs filtrées :");

    noInterrupts();
    uint16_t snapshot[10];
    int currentIndex = indexBuffer;
    for (int i = 0; i < 10; i++) {
      int pos = (currentIndex - 10 + i + BUFFER_SIZE) % BUFFER_SIZE;
      snapshot[i] = buffer[pos];
    }
    interrupts();

    for (int i = 0; i < 10; i++) {
      float filtered = applyFIR((float)snapshot[i]);
      if (filtered < 0) filtered = 0;
      if (filtered > 4095) filtered = 4095;
      Serial.println((uint16_t)filtered);
    }
  }
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR; // Acquitte l’interruption

  buffer[indexBuffer] = ADC->ADC_CDR[7];
  indexBuffer = (indexBuffer + 1) % BUFFER_SIZE;

  ADC->ADC_CR = ADC_CR_START;
}
