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

#define MAX_TONES 3

#define GAIN_SLEW_PER_SAMPLE 50.0f
#define PHASE_STEP_COARSE  (M_PI / 18.0)   // 10 deg
#define PHASE_STEP_FINE    (M_PI / 180.0)  // 1 deg
#define DWELL_MS           1200
#define DONE_BEEP_FREQ     400.0f
#define DONE_BEEP_MS       300
#define PLAYBACK_ATTEN     0.5f

// measurement hygiene
#define DISCARD_MS         100
#define AVG_GAP_MS         50
#define AFTER_WAIT_MS      800
#define MIN_BIN            10              // avoid DC and very low bins
#define MIN_SEP_BINS       6               // separation of peaks in bins
#define GAIN_ITER_MAX      10
#define GAIN_TOL_LOW       0.90f
#define GAIN_TOL_HIGH      1.10f

// FFT objects
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, SAMPLE_RATE);

// I2S buffers
int32_t sampleBuffer[BUFFER_SIZE];
int32_t outBuffer[CHUNK_SAMPLES];

// Multi-tone params
float toneFreqs[MAX_TONES];
float tonePhases[MAX_TONES];
float tonePhaseIncs[MAX_TONES];
float toneOffsets[MAX_TONES];     // calibrated phase offsets
float targetGains[MAX_TONES];     // digital gains per tone (speaker domain)
float currentGains[MAX_TONES];    // slewed toward targetGains
bool  toneActive[MAX_TONES];      // during calibration we enable one at a time
int   numTones = 0;

// For reporting and weighting
float drillAmps[MAX_TONES];       // mic amplitudes per peak (drill domain)

// Legacy single-tone vars kept for compatibility with prefs
float legacy_freq  = 7000.0f;
float legacy_phase = 0.0f;
float legacy_gain  = 1000.0f;

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

static inline void discardAndWarmUpRead() {
  size_t bytesRead;
  i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);
  delay(DISCARD_MS);
}

int freqToBin(float f) {
  int bin = (int)roundf(f * BUFFER_SIZE / SAMPLE_RATE);
  if (bin < 1) bin = 1;
  if (bin >= BUFFER_SIZE / 2) bin = BUFFER_SIZE / 2 - 1;
  return bin;
}

float binToFreq(int bin) {
  return (float)bin * SAMPLE_RATE / BUFFER_SIZE;
}

void computePhaseIncs() {
  for (int t = 0; t < numTones; t++) {
    tonePhaseIncs[t] = (2.0f * (float)M_PI * toneFreqs[t]) / SAMPLE_RATE;
  }
}

void setAllTonesInactive() {
  for (int t = 0; t < MAX_TONES; t++) toneActive[t] = false;
}

void enableOnlyTone(int idx) {
  setAllTonesInactive();
  if (idx >= 0 && idx < numTones) toneActive[idx] = true;
}

void enableAllTones() {
  for (int t = 0; t < numTones; t++) toneActive[t] = true;
}

// -------------------- Audio generation --------------------
void generateAndWriteBlock() {
  for (int i = 0; i < CHUNK_SAMPLES; i++) {
    float mix = 0.0f;

    // smooth per-tone gains and accumulate
    for (int t = 0; t < numTones; t++) {
      if (!toneActive[t]) continue;

      float delta = targetGains[t] - currentGains[t];
      if (fabsf(delta) > GAIN_SLEW_PER_SAMPLE) {
        currentGains[t] += GAIN_SLEW_PER_SAMPLE * (delta > 0 ? 1.0f : -1.0f);
      } else {
        currentGains[t] = targetGains[t];
      }

      float s = sinf(tonePhases[t] + toneOffsets[t]) * currentGains[t];
      mix += s;
      tonePhases[t] = wrap2pi(tonePhases[t] + tonePhaseIncs[t]);
    }

    int16_t val16 = (int16_t)(mix * PLAYBACK_ATTEN);
    outBuffer[i] = (int32_t)val16 << 16;  // left channel only
  }

  size_t bw;
  i2s_write(I2S_NUM_0, (char *)outBuffer, sizeof(outBuffer), &bw, portMAX_DELAY);
}

// -------------------- Beep --------------------
void beep(float f, int ms) {
  float inc = (2.0f * (float)M_PI * f) / SAMPLE_RATE;
  float ph = 0.0f;

  int totalSamples = (SAMPLE_RATE * ms) / 1000;
  while (totalSamples > 0) {
    int n = (totalSamples > CHUNK_SAMPLES ? CHUNK_SAMPLES : totalSamples);
    for (int i = 0; i < n; i++) {
      float s = sinf(ph);
      ph = wrap2pi(ph + inc);
      int16_t val16 = (int16_t)(s * 12000.0f);
      outBuffer[i] = (int32_t)val16 << 16;
    }
    size_t bw;
    i2s_write(I2S_NUM_0, (char *)outBuffer, n * sizeof(int32_t), &bw, portMAX_DELAY);
    totalSamples -= n;
  }
}

// -------------------- FFT utilities --------------------
void acquireFFTOnce() {
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
}

void dumpSpectrumCSV(const char* tag) {
  acquireFFTOnce();
  for (int i = MIN_BIN; i < BUFFER_SIZE / 2; i += 2) {
    float f = (float)i * SAMPLE_RATE / BUFFER_SIZE;
    Serial.printf("%s,%.0f,%.3f\n", tag, f, vReal[i]);
  }
}

float measureAtFrequency(float f, int averages = 3) {
  float sum = 0.0f;
  int bin = freqToBin(f);
  size_t bytesRead;

  discardAndWarmUpRead();
  for (int a = 0; a < averages; a++) {
    i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);
    for (int i = 0; i < BUFFER_SIZE; i++) {
      vReal[i] = (double)sampleBuffer[i];
      vImag[i] = 0.0;
    }
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    sum += (float)vReal[bin];
    delay(AVG_GAP_MS);
  }
  return sum / (float)averages;
}

// Find top N separated peaks from a single FFT capture
int detectTopPeaks(float *freqsOut, float *ampsOut, int maxPeaks) {
  acquireFFTOnce();

  struct Peak { int bin; float val; };
  Peak top[MAX_TONES];
  for (int p = 0; p < MAX_TONES; p++) { top[p].bin = -1; top[p].val = 0.0f; }

  int half = BUFFER_SIZE / 2;
  for (int i = MIN_BIN; i < half; i++) {
    float val = (float)vReal[i];
    // maintain a small local maximum check
    if (i > MIN_BIN && i < half - 1) {
      if (val < vReal[i - 1] || val < vReal[i + 1]) continue;
    }

    // try to insert into top[] if sufficiently separated
    for (int p = 0; p < maxPeaks; p++) {
      if (val > top[p].val) {
        // check separation
        bool ok = true;
        for (int q = 0; q < maxPeaks; q++) {
          if (top[q].bin >= 0 && abs(i - top[q].bin) < MIN_SEP_BINS) {
            ok = false; break;
          }
        }
        if (!ok) break;

        // shift down
        for (int s = maxPeaks - 1; s > p; s--) top[s] = top[s - 1];
        top[p].bin = i;
        top[p].val = val;
        break;
      }
    }
  }

  int found = 0;
  for (int p = 0; p < maxPeaks; p++) {
    if (top[p].bin >= 0) {
      freqsOut[found] = binToFreq(top[p].bin);
      ampsOut[found]  = top[p].val;
      found++;
    }
  }
  return found;
}

// -------------------- Gain Matching Helper (per tone) --------------------
float matchSpeakerAmplitude(float drillAmp, float freq) {
  Serial.println("=== MATCHING SPEAKER AMPLITUDE ===");
  float testGain = 2000.0f;

  outputEnabled = true;
  delay(300);

  for (int iter = 0; iter < GAIN_ITER_MAX; iter++) {
    // apply and settle
    // the caller should ensure only this tone is active
    for (int t = 0; t < numTones; t++) {
      if (toneActive[t]) targetGains[t] = testGain;
    }
    delay(300);
    float measuredAmp = measureAtFrequency(freq, 3);
    float ratio = measuredAmp / (drillAmp + 1e-9f);

    Serial.printf("GainMatch: target=%.1f  meas=%.3f  drill=%.3f  ratio=%.3f\n",
                  testGain, measuredAmp, drillAmp, ratio);

    if (ratio > GAIN_TOL_LOW && ratio < GAIN_TOL_HIGH) break;
    if (ratio > GAIN_TOL_HIGH) testGain *= 0.85f;
    else                       testGain *= 1.15f;

    if (testGain < 800.0f)   testGain = 800.0f;
    if (testGain > 28000.0f) testGain = 28000.0f;
  }

  outputEnabled = false;
  Serial.printf("Matched Gain: %.1f\n", testGain);
  return testGain;
}

// -------------------- Phase Calibration Helper (per tone) --------------------
float calibratePhaseAtFrequency(float freq, float &bestAmpOut) {
  float bestPh  = 0.0f;
  float bestAmp = 1e12;

  // coarse sweep
  for (float ph = -M_PI; ph <= M_PI; ph += PHASE_STEP_COARSE) {
    for (int t = 0; t < numTones; t++) if (toneActive[t]) toneOffsets[t] = ph;
    outputEnabled = true;
    delay(DWELL_MS);
    float amp = measureAtFrequency(freq, 3);
    Serial.printf("COARSE,%.2f,%.3f\n", ph, amp);
    if (amp < bestAmp) { bestAmp = amp; bestPh = ph; }
    outputEnabled = false;
  }

  // fine sweep around best
  for (float ph = bestPh - 0.5f; ph <= bestPh + 0.5f; ph += PHASE_STEP_FINE) {
    for (int t = 0; t < numTones; t++) if (toneActive[t]) toneOffsets[t] = ph;
    outputEnabled = true;
    delay(DWELL_MS);
    float amp = measureAtFrequency(freq, 3);
    Serial.printf("FINE,%.2f,%.3f\n", ph, amp);
    if (amp < bestAmp) { bestAmp = amp; bestPh = ph; }
    outputEnabled = false;
  }

  bestAmpOut = bestAmp;
  return bestPh;
}

// -------------------- Calibration --------------------
void runCalibration() {
  calibrating = true;
  Serial.println("=== CALIBRATION START ===");

  // stop output, record baseline spectrum
  outputEnabled = false;
  delay(120);
  dumpSpectrumCSV("BEFORE");

  // find top peaks and their amplitudes
  float freqs[MAX_TONES];
  float amps[MAX_TONES];
  int found = detectTopPeaks(freqs, amps, MAX_TONES);

  if (found <= 0) {
    Serial.println("No peaks detected. Aborting calibration.");
    calibrating = false;
    return;
  }

  // normalize amplitudes into weights that sum to 1
  float sumAmp = 0.0f;
  for (int i = 0; i < found; i++) sumAmp += amps[i];
  for (int i = 0; i < found; i++) {
    if (sumAmp > 0) amps[i] /= sumAmp; else amps[i] = (i == 0 ? 1.0f : 0.0f);
  }

  // initialize tones using detected peaks
  numTones = found;
  for (int t = 0; t < numTones; t++) {
    toneFreqs[t]   = freqs[t];
    drillAmps[t]   = 0.0f;     // will be measured per-tone below
    tonePhases[t]  = 0.0f;
    toneOffsets[t] = 0.0f;
    currentGains[t]= 0.0f;
    targetGains[t] = 0.0f;
    toneActive[t]  = false;
  }
  computePhaseIncs();

  // measure real drill amplitude at each tone frequency for ground truth
  // do this with output off
  for (int t = 0; t < numTones; t++) {
    drillAmps[t] = measureAtFrequency(toneFreqs[t], 5);
    Serial.printf("Peak %d: f=%.2f Hz  drillAmp=%.4f  weight=%.3f\n",
                  t, toneFreqs[t], drillAmps[t], (sumAmp > 0 ? (drillAmps[t] / (drillAmps[0] + drillAmps[1] + drillAmps[2] + 1e-9f)) : 0.0f));
  }

  // amplitude matching and phase calibration per tone
  for (int t = 0; t < numTones; t++) {
    // desired amplitude is the actual measured drill amplitude at that bin
    float desiredAmp = drillAmps[t];

    // enable only this tone for matching and phase search
    enableOnlyTone(t);

    // 1) gain match
    targetGains[t] = matchSpeakerAmplitude(desiredAmp, toneFreqs[t]);

    // 2) phase calibration at this frequency
    float bestAmp = 0.0f;
    float bestPh  = calibratePhaseAtFrequency(toneFreqs[t], bestAmp);
    toneOffsets[t] = bestPh;

    // 3) auto-flip if needed
    outputEnabled = false;
    delay(200);
    float ampAfter = measureAtFrequency(toneFreqs[t], 5);
    if (ampAfter > desiredAmp) {
      Serial.println("Detected constructive interference on this tone. Flipping phase.");
      toneOffsets[t] = wrap2pi(toneOffsets[t] + (float)M_PI);
    }

    // keep this tone configured but inactive while we calibrate others
    toneActive[t] = false;
    Serial.printf("Calibrated tone %d: f=%.2f Hz  gain=%.1f  phase=%.3f rad\n",
                  t, toneFreqs[t], targetGains[t], toneOffsets[t]);
  }

  // after individual calibration, enable all tones
  enableAllTones();
  outputEnabled = true;

  // settle then measure final amplitudes at each peak
  delay(AFTER_WAIT_MS);
  outputEnabled = false;
  float totalBefore = 0.0f, totalAfter = 0.0f;
  for (int t = 0; t < numTones; t++) totalBefore += drillAmps[t];

  for (int t = 0; t < numTones; t++) {
    float ampAfter = measureAtFrequency(toneFreqs[t], 5);
    totalAfter += ampAfter;
    float dropdB = 20.0f * log10f((drillAmps[t] + 1e-9f) / (ampAfter + 1e-9f));
    Serial.printf("Tone %d result: f=%.2f Hz  before=%.4f  after=%.4f  change=%.2f dB\n",
                  t, toneFreqs[t], drillAmps[t], ampAfter, dropdB);
  }

  float netDrop = 20.0f * log10f((totalBefore + 1e-9f) / (totalAfter + 1e-9f));
  Serial.printf("NET change across peaks: %.2f dB\n", netDrop);

  Serial.println("=== CALIBRATION COMPLETE ===");
  dumpSpectrumCSV("AFTER");

  // save minimal state for fast boot
  prefs.begin("antiphonon", false);
  prefs.putInt("numTones", numTones);
  for (int t = 0; t < numTones; t++) {
    char keyF[8], keyP[8], keyG[8];
    snprintf(keyF, sizeof(keyF), "f%d", t);
    snprintf(keyP, sizeof(keyP), "p%d", t);
    snprintf(keyG, sizeof(keyG), "g%d", t);
    prefs.putFloat(keyF, toneFreqs[t]);
    prefs.putFloat(keyP, toneOffsets[t]);
    prefs.putFloat(keyG, targetGains[t]);
  }
  // also store legacy primary for compatibility
  prefs.putFloat("freq",  toneFreqs[0]);
  prefs.putFloat("phase", toneOffsets[0]);
  prefs.putFloat("gain",  targetGains[0]);
  prefs.end();

  outputEnabled = true;
  beep(DONE_BEEP_FREQ, DONE_BEEP_MS);
  calibrating = false;
}

// -------------------- Background audio task --------------------
void audioTask(void *param) {
  for (;;) {
    if (outputEnabled) {
      generateAndWriteBlock();
    } else {
      vTaskDelay(1);
    }
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // load saved params
  prefs.begin("antiphonon", true);
  int savedN = prefs.getInt("numTones", 1);
  if (savedN < 1) savedN = 1;
  if (savedN > MAX_TONES) savedN = MAX_TONES;

  numTones = savedN;
  for (int t = 0; t < numTones; t++) {
    char keyF[8], keyP[8], keyG[8];
    snprintf(keyF, sizeof(keyF), "f%d", t);
    snprintf(keyP, sizeof(keyP), "p%d", t);
    snprintf(keyG, sizeof(keyG), "g%d", t);
    toneFreqs[t]   = prefs.getFloat(keyF, legacy_freq);
    toneOffsets[t] = prefs.getFloat(keyP, legacy_phase);
    targetGains[t] = prefs.getFloat(keyG, legacy_gain);
    tonePhases[t]  = 0.0f;
    currentGains[t]= 0.0f;
    toneActive[t]  = true; // enable by default
  }
  // legacy backup if prefs were empty
  if (numTones == 1 && toneFreqs[0] == legacy_freq && targetGains[0] == legacy_gain) {
    toneOffsets[0] = prefs.getFloat("phase", 0.0f);
    toneFreqs[0]   = prefs.getFloat("freq",  7000.0f);
    targetGains[0] = prefs.getFloat("gain",  1000.0f);
  }
  prefs.end();

  computePhaseIncs();

  // start audio loop on core 0
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
