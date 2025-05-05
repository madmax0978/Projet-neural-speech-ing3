#define ADC_PIN A0                 // Entrée micro MAX9814 = canal ADC7
#define BUFFER_SIZE 1024          // Taille du buffer circulaire

volatile uint16_t buffer[BUFFER_SIZE];
volatile uint16_t indexBuffer = 0;
volatile uint16_t readIndex = 0;

void setup() {
  Serial.begin(115200);

  // --- Configuration de l'ADC (12 bits, canal 7 = A0) ---
  analogReadResolution(12);
  ADC->ADC_MR &= ~ADC_MR_LOWRES;
  ADC->ADC_CHER = ADC_CHER_CH7;
  ADC->ADC_CR = ADC_CR_START;

  // --- Configuration du DAC0 ---
  analogWriteResolution(12);                 // Résolution 12 bits
  pmc_enable_periph_clk(ID_DACC);            // Active l’horloge du DAC
  dacc_enable_channel(DACC, 0);              // Active DAC0 (canal 0)

  // --- Timer TC0 canal 0 pour échantillonnage ADC à 32 kHz ---
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC0->TC_CHANNEL[0].TC_CMR =
      TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
  uint32_t rc = 42000000 / 32000;
  TC0->TC_CHANNEL[0].TC_RC = rc;           
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  NVIC_EnableIRQ(TC0_IRQn);

  // --- Timer TC1 canal 0 pour sortie DAC à 32 kHz ---
  pmc_enable_periph_clk(ID_TC3);             // TC1 canal 0 = TC3
  TC1->TC_CHANNEL[0].TC_CMR =
      TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
  TC1->TC_CHANNEL[0].TC_RC = 1312;
  TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  NVIC_EnableIRQ(TC3_IRQn);
}

void loop() {
  // Affiche 10 dernières valeurs toutes les secondes pour contrôle
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

// --- Interruption Timer TC0 (ADC à 32 kHz) ---
void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;              // Acquitte interruption
  buffer[indexBuffer] = ADC->ADC_CDR[7]; // Lire l’échantillon ADC
  indexBuffer = (indexBuffer + 1) % BUFFER_SIZE;
  ADC->ADC_CR = ADC_CR_START;            // Relancer conversion
}

// --- Interruption Timer TC1 (sortie DAC à 32 kHz) ---
void TC3_Handler() {
  TC1->TC_CHANNEL[0].TC_SR;              // Acquitte interruption
  dacc_write_conversion_data(DACC, buffer[readIndex]); // Vers DAC0
  readIndex = (readIndex + 1) % BUFFER_SIZE;
}
