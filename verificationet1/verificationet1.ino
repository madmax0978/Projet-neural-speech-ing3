#define ADC_PIN A0
#define BUFFER_SIZE 1024

volatile uint16_t buffer[BUFFER_SIZE];
volatile uint16_t indexBuffer = 0;

void setup() {
  Serial.begin(115200);

  // --- Configuration ADC ---
  analogReadResolution(12);
  ADC->ADC_MR &= ~ADC_MR_LOWRES;
  ADC->ADC_CHER = ADC_CHER_CH7;
  ADC->ADC_CR = ADC_CR_START;

  // --- Configuration DAC0 ---
  analogWriteResolution(12); // DAC en 12 bits

  // --- Timer TC0 à 32 kHz ---
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR =
      TC_CMR_TCCLKS_TIMER_CLOCK1
    | TC_CMR_WAVE
    | TC_CMR_WAVSEL_UP_RC;

  uint32_t rc = 42000000 / 32000;
  TC0->TC_CHANNEL[0].TC_RC = rc;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  NVIC_EnableIRQ(TC0_IRQn);
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.println("10 dernières valeurs :");
    for (int i = 0; i < 10; i++) {
      int pos = (indexBuffer - 10 + i + BUFFER_SIZE) % BUFFER_SIZE;
      Serial.println(buffer[pos]);
    }
  }
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;

  uint16_t val = ADC->ADC_CDR[7];
  buffer[indexBuffer] = val;
  indexBuffer = (indexBuffer + 1) % BUFFER_SIZE;

  analogWrite(DAC0, val); // Sortie sur DAC0

  ADC->ADC_CR = ADC_CR_START;
}
