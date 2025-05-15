#define ADC_PIN A0
#define BUFFER_SIZE 1024
#define RIF_TAPS 13
#define RECORD_SIZE 8000     // 1 seconde à 8 kHz
#define BUTTON_PIN 22        // Bouton sur pin 2

volatile uint16_t adcRaw = 0;
volatile bool sampleReady = false;

float firCoeffs[RIF_TAPS] = {
  // avec 21 valeur : 0.002536, 0.002553, -0.000000, -0.008635, -0.021022, -0.024211, 0.000000,0.060557, 0.144592, 0.219131, 0.248999, 0.219131, 0.144592, 0.060557,0.000000, -0.024211, -0.021022, -0.008635, -0.000000, 0.002553, 0.002536
  -0.009191, 0.004612, 0.031870, 0.082049, 0.139093, 0.182848, 
  0.197318, 0.182848, 0.139093, 0.082049, 0.031870, 0.004612, -0.009191
};

float firBuffer[RIF_TAPS] = {0};
uint16_t recordBuffer[RECORD_SIZE];
bool recording = false;
int recordedSamples = 0;
int downsampleCounter = 0;

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
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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
}

void loop() {
  // Démarrage de l'enregistrement à la pression du bouton
  if (!recording && digitalRead(BUTTON_PIN) == LOW) {
    delay(50); // anti-rebond
    if (digitalRead(BUTTON_PIN) == HIGH) {
      recordedSamples = 0;
      recording = true;
      Serial.println("== DEBUT ENREGISTREMENT ==");
    }
  }

  if (recording && sampleReady) {
    sampleReady = false;

    if (++downsampleCounter == 4) {  // 32kHz → 8kHz
      downsampleCounter = 0;
      float filtered = applyFIR((float)adcRaw);
      if (filtered < 0) filtered = 0;
      if (filtered > 4095) filtered = 4095;

      recordBuffer[recordedSamples++] = (uint16_t)filtered;

      if (recordedSamples >= RECORD_SIZE) {
        recording = false;
        Serial.println("== FIN ENREGISTREMENT ==");
        Serial.println("== EXPORT VERS AUDACITY ==");

        // Envoie tous les échantillons pour copier/coller dans Audacity
        for (int i = 0; i < RECORD_SIZE; i++) {
          Serial.println(recordBuffer[i]);
        }
        Serial.println("== FIN EXPORT ==");
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
