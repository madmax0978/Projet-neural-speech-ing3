#define ADC_PIN A0                 // Entrée micro MAX9814 = canal ADC7
#define BUFFER_SIZE 1024          // Taille du buffer circulaire

volatile uint16_t buffer[BUFFER_SIZE];
volatile uint16_t indexBuffer = 0;

void setup() {
  Serial.begin(115200);

  // --- Configuration de l'ADC (en 12 bits, canal 7 = A0) ---
  analogReadResolution(12);                 // Interface Arduino
  ADC->ADC_MR &= ~ADC_MR_LOWRES;            // Résolution 12 bits (0–4095)
  ADC->ADC_CHER = ADC_CHER_CH7;             // Active le canal 7

  // --- Lancer une première conversion pour amorcer ---
  ADC->ADC_CR = ADC_CR_START;

  // --- Configuration du Timer TC0, canal 0 pour 32kHz ---
  pmc_set_writeprotect(false);              // Autorise écriture sur registre PMC
  pmc_enable_periph_clk(ID_TC0);            // Active horloge pour TC0

  TC0->TC_CHANNEL[0].TC_CMR =
      TC_CMR_TCCLKS_TIMER_CLOCK1            // Clock = MCK/2 = 42 MHz
    | TC_CMR_WAVE                           // Mode onde
    | TC_CMR_WAVSEL_UP_RC;                 // Reset counter sur RC match

  uint32_t rc = 42000000 / 32000;           // 42 MHz / 32 kHz = 1312.5 → 1312
  TC0->TC_CHANNEL[0].TC_RC = rc;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;  // Interruption sur RC
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  NVIC_EnableIRQ(TC0_IRQn);                 // Active l’interruption TC0
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

// --- Interruption : lecture ADC et stockage circulaire ---
void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR;              // Acquitte l’interruption
  buffer[indexBuffer] = ADC->ADC_CDR[7]; // Lit conversion du canal 7 (A0)
  indexBuffer = (indexBuffer + 1) % BUFFER_SIZE;
  ADC->ADC_CR = ADC_CR_START;            // Relance la conversion
}
