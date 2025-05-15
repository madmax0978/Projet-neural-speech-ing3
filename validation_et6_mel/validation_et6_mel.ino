#include <arduinoMFCC.h>

#define ADC_PIN A0
#define BUFFER_SIZE 1024
#define RIF_TAPS 53
#define RECORD_SIZE 8000     // 1 seconde à 8 kHz
#define BUTTON_PIN 22        // Bouton sur pin 22

#define FRAME_SIZE 256
#define HOP_SIZE 128
#define NUM_FRAMES ((RECORD_SIZE - FRAME_SIZE) / HOP_SIZE + 1)  // 61 frames

#define MFCC_SIZE 26
#define DCT_SIZE 13

uint16_t frames[NUM_FRAMES][FRAME_SIZE];
float mfccMatrix[NUM_FRAMES][DCT_SIZE];  // 61 lignes × 13 colonnes

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
      if (start + j < RECORD_SIZE) {
        frames[i][j] = recordBuffer[start + j];
      } else {
        frames[i][j] = 0;
      }
    }
  }
}

void computeMFCCs() {
  Serial.println("== CALCUL MFCCs (AVANT NORMALISATION) ==");
  for (int i = 0; i < NUM_FRAMES; i++) {
    float inputFrame[FRAME_SIZE];
    for (int j = 0; j < FRAME_SIZE; j++) {
      inputFrame[j] = (float)frames[i][j];
    }

    float mfccResult[DCT_SIZE];
    mfcc.computeWithDCT(inputFrame, mfccResult);

    // Sauvegarde dans matrice
    for (int k = 0; k < DCT_SIZE; k++) {
      mfccMatrix[i][k] = mfccResult[k];
    }

    // Export des coeffs MFCC dans le terminal (fichier mfcc.txt)
    for (int k = 0; k < DCT_SIZE; k++) {
      Serial.println(mfccResult[k], 4);  // Une ligne par valeur
    }
  }
}

void normalizeMFCCs() {
  Serial.println("== NORMALISATION ==");

  for (int col = 0; col < DCT_SIZE; col++) {
    float sum = 0;
    for (int row = 0; row < NUM_FRAMES; row++) {
      sum += mfccMatrix[row][col];
    }
    float mean = sum / NUM_FRAMES;

    float variance = 0;
    for (int row = 0; row < NUM_FRAMES; row++) {
      float diff = mfccMatrix[row][col] - mean;
      variance += diff * diff;
    }
    float stddev = sqrt(variance / NUM_FRAMES);

    for (int row = 0; row < NUM_FRAMES; row++) {
      mfccMatrix[row][col] = (mfccMatrix[row][col] - mean) / stddev;
    }
  }
}

void printMFCCs() {
  Serial.println("== MFCCs Normalisés ==");
  for (int i = 0; i < NUM_FRAMES; i++) {
    Serial.print("Frame ");
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < DCT_SIZE; j++) {
      Serial.print(mfccMatrix[i][j], 4);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void computeMELOnly() {
  Serial.println("== MEL COEFFS ==");
  for (int i = 0; i < NUM_FRAMES; i++) {
    float inputFrame[FRAME_SIZE];
    for (int j = 0; j < FRAME_SIZE; j++) {
      inputFrame[j] = (float)frames[i][j];
    }

    // Copie du frame dans l’objet MFCC (mais pas de DCT)
    mfcc.compute(inputFrame, nullptr); // ça met à jour mfcc._mfcc_coeffs

    // Affiche chaque coefficient MEL (26 en tout)
    for (int k = 0; k < MFCC_SIZE; k++) {
      Serial.println(mfcc._mfcc_coeffs[k]);
    }
  }
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

  // Initialisation MFCC
  mfcc.create_hamming_window();
  mfcc.create_mel_filter_bank();
  mfcc.create_dct_matrix();
}

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
        Serial.println("== EXPORT VERS AUDACITY ==");
        for (int i = 0; i < RECORD_SIZE; i++) {
          Serial.println(recordBuffer[i]);
        }
        Serial.println("== FIN EXPORT ==");
        extractFrames();


        // ET6 : MFCC validation
        computeMELOnly(); // à la place du calcul DCT

        //calcul des valeurs apres le DCT pour comparer avec les valeurs à la sortie du mel 
        computeMFCCs();
        printMFCCs();


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
