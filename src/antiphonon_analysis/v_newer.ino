#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>
#include <Preferences.h>

// -------------------- Pins and constants --------------------
#define BCLK_PIN    26
#define LRCK_PIN    25
#define SPK_DIN     22
#define MIC_DOUT    33
#define CAL_BUTTON  5

#define SAMPLE_RATE     22050
#define BUFFER_SIZE     2048
#define CHUNK_SAMPLES   1024

#define GAIN_SLEW_PER_SAMPLE 50.0f
#define PHASE_STEP_COARSE  (M_PI / 18.0)
#define PHASE_STEP_FINE    (M_PI / 180.0)
#define DWELL_MS           1200
#define DONE_BEEP_FREQ     400.0f
#define DONE_BEEP_MS       300
#define PLAYBACK_ATTEN     0.5f

#define DISCARD_MS         100
#define AVG_GAP_MS         50

// FFT objects
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, SAMPLE_RATE);

// I2S buffers
int32_t sampleBuffer[BUFFER_SIZE];
int32_t outBuffer[CHUNK_SAMPLES];

// Tone parameters
float toneFreq    = 7000.0f;
float tonePhase   = 0.0f;
float tonePhaseInc= 0.0f;
float phaseOffset = 0.0f;

float currentGain = 0.0f;
float targetGain  = 1000.0f;

volatile bool outputEnabled = true;
volatile bool calibrating   = false;

Preferences prefs;

// -------------------- I2S config --------------------
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = 0,
  .dma_buf_count = 8,
  .dma_buf_len = CHUNK_SAMPLES,
  .use_apll = false,
  .tx_desc_auto_clear = true,
  .fixed_mclk = 0
};

i2s_pin_config_t pin_config = {
  .bck_io_num   = BCLK_PIN,
  .ws_io_num    = LRCK_PIN,
  .data_out_num = SPK_DIN,
  .data_in_num  = MIC_DOUT
};

// -------------------- Helpers --------------------
float wrap2pi(float x) {
  if (x >= 2.0f * (float)M_PI) x -= 2.0f * (float)M_PI;
  else if (x < 0.0f) x += 2.0f * (float)M_PI;
  return x;
}

void computePhaseIncrement() {
  tonePhaseInc = (2.0f * (float)M_PI * toneFreq) / SAMPLE_RATE;
}

static inline void discardAndWarmUpRead() {
  size_t bytesRead;
  i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);
  delay(DISCARD_MS);
}

// -------------------- Audio generation --------------------
void generateAndWriteBlock() {
  for (int i = 0; i < CHUNK_SAMPLES; i++) {
    float s = sinf(tonePhase + phaseOffset);
    tonePhase = wrap2pi(tonePhase + tonePhaseInc);

    float delta = targetGain - currentGain;
    if (fabsf(delta) > GAIN_SLEW_PER_SAMPLE)
      currentGain += GAIN_SLEW_PER_SAMPLE * (delta > 0 ? 1.0f : -1.0f);
    else
      currentGain = targetGain;

    int16_t val16 = (int16_t)(s * currentGain * PLAYBACK_ATTEN);
    outBuffer[i] = (int32_t)val16 << 16;
  }

  size_t bw;
  i2s_write(I2S_NUM_0, (char *)outBuffer, sizeof(outBuffer), &bw, portMAX_DELAY);
}

// -------------------- FFT utilities --------------------
float measureAtFrequency(float f, int averages = 3) {
  discardAndWarmUpRead();
  float sum = 0.0f;
  int bin = (int)(f * BUFFER_SIZE / SAMPLE_RATE);
  size_t bytesRead;

  for (int a = 0; a < averages; a++) {
    i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);
    for (int i = 0; i < BUFFER_SIZE; i++) {
      vReal[i] = (double)sampleBuffer[i];
      vImag[i] = 0.0;
    }
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    sum += vReal[bin];
    delay(AVG_GAP_MS);
  }
  return sum / (float)averages;
}

float detectDominantFrequency(float &ampOut) {
  size_t bytesRead;
  discardAndWarmUpRead();
  i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

  for (int i = 0; i < BUFFER_SIZE; i++) {
    vReal[i] = (double)sampleBuffer[i];
    vImag[i] = 0.0;
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  int peakBin = 1;
  double peakVal = vReal[1];

  for (int i = 1; i < BUFFER_SIZE / 2; i++) {
    if (vReal[i] > peakVal) {
      peakVal = vReal[i];
      peakBin = i;
    }
  }

  float freq = (float)peakBin * SAMPLE_RATE / BUFFER_SIZE;
  ampOut = (float)peakVal;

  Serial.printf("FFT Peak: %.2f Hz | Amp: %.3f\n", freq, ampOut);
  return freq;
}

float matchSpeakerAmplitude(float drillAmp, float freq) {
  Serial.println("=== MATCHING SPEAKER AMPLITUDE ===");
  float testGain = 2000.0f;
  float measuredAmp = 0.0f;

  for (int iter = 0; iter < 10; iter++) {
    targetGain = testGain;
    delay(300);
    measuredAmp = measureAtFrequency(freq, 3);
    float ratio = measuredAmp / (drillAmp + 1e-9f);
    Serial.printf("GainMatch Iter %d: Gain=%.1f | Amp=%.3f | Ratio=%.3f\n",
                  iter + 1, testGain, measuredAmp, ratio);

    if (ratio > 0.9f && ratio < 1.1f) break;
    if (ratio > 1.1f) testGain *= 0.85f;
    else testGain *= 1.15f;
    if (testGain < 1000.0f) testGain = 1000.0f;
    if (testGain > 25000.0f) testGain = 25000.0f;
  }
  Serial.printf("Matched Gain: %.1f\n", testGain);
  return testGain;
}

// -------------------- Calibration --------------------
void runCalibration() {
  calibrating = true;
  Serial.println("=== CALIBRATION START ===");

  // Speaker off initially for clean drill detection
  outputEnabled = false;
  delay(200);
  float ampBefore = 0.0f;
  float detectedFreq = detectDominantFrequency(ampBefore);

  toneFreq = detectedFreq;
  computePhaseIncrement();

  // Start continuous playback
  outputEnabled = true;
  delay(200);

  // Match amplitude before phase sweep
  targetGain = matchSpeakerAmplitude(ampBefore, toneFreq);

  float bestPh = 0.0f;
  float bestAmp = 1e12;

  // Coarse sweep
  for (float ph = -M_PI; ph <= M_PI; ph += PHASE_STEP_COARSE) {
    phaseOffset = ph;
    delay(DWELL_MS);
    float amp = measureAtFrequency(toneFreq, 3);
    Serial.printf("COARSE,%.2f,%.2f\n", ph, amp);
    if (amp < bestAmp) { bestAmp = amp; bestPh = ph; }
  }

  // Fine sweep
  for (float ph = bestPh - 0.5f; ph <= bestPh + 0.5f; ph += PHASE_STEP_FINE) {
    phaseOffset = ph;
    delay(DWELL_MS);
    float amp = measureAtFrequency(toneFreq, 3);
    Serial.printf("FINE,%.2f,%.2f\n", ph, amp);
    if (amp < bestAmp) { bestAmp = amp; bestPh = ph; }
  }

  // Apply optimal phase continuously and let it stabilize for 1s
  phaseOffset = bestPh;
  Serial.printf("Best Phase Found: %.3f rad. Letting field stabilize...\n", bestPh);
  delay(1000);

  // Measure after stabilization
  float ampAfter = measureAtFrequency(toneFreq, 5);
  float dropdB = 20.0f * log10f((ampBefore + 1e-9f) / (ampAfter + 1e-9f));

  if (ampAfter > ampBefore) {
    Serial.println(">>> Amplitude increased — flipping phase 180°");
    phaseOffset = wrap2pi(phaseOffset + M_PI);
    delay(1000);
    ampAfter = measureAtFrequency(toneFreq, 5);
    dropdB = 20.0f * log10f((ampBefore + 1e-9f) / (ampAfter + 1e-9f));
  }

  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.printf("Frequency: %.2f Hz\n", toneFreq);
  Serial.printf("Initial Amp: %.4f\n", ampBefore);
  Serial.printf("Final Amp:   %.4f\n", ampAfter);
  Serial.printf("Change:      %.2f dB\n", dropdB);
  Serial.printf("Optimal Phase Offset: %.3f rad\n", phaseOffset);

  prefs.begin("antiphonon", false);
  prefs.putFloat("freq", toneFreq);
  prefs.putFloat("phase", phaseOffset);
  prefs.putFloat("gain", targetGain);
  prefs.end();

  // beep(DONE_BEEP_FREQ, DONE_BEEP_MS);
  calibrating = false;
}

// -------------------- Background audio task --------------------
void audioTask(void *param) {
  for (;;) {
    if (outputEnabled) generateAndWriteBlock();
    else vTaskDelay(1);
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  pinMode(CAL_BUTTON, INPUT_PULLUP);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  prefs.begin("antiphonon", true);
  toneFreq = prefs.getFloat("freq", 7000.0f);
  phaseOffset = prefs.getFloat("phase", 0.0f);
  targetGain = prefs.getFloat("gain", 1000.0f);
  prefs.end();

  computePhaseIncrement();
  xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, NULL, 1, NULL, 0);

  Serial.println("ANTIPHONON ready. Hold drill near mic and press CAL.");
}

// -------------------- Loop --------------------
void loop() {
  if (!calibrating && digitalRead(CAL_BUTTON) == LOW) {
    runCalibration();
    delay(400);
  }
}
