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

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 2048
#define CHUNK_SAMPLES 1024

#define GAIN_SLEW_PER_SAMPLE 0.0005f
#define PHASE_STEP_COARSE  (M_PI / 18.0)   // 10 deg
#define PHASE_STEP_FINE    (M_PI / 180.0)  // 1 deg
#define DWELL_MS           300
#define DONE_BEEP_FREQ     400.0f
#define DONE_BEEP_MS       300
#define PLAYBACK_ATTEN     0.70f

// FFT objects
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, SAMPLE_RATE);

// I2S buffers
int32_t sampleBuffer[BUFFER_SIZE];
int32_t outBuffer[CHUNK_SAMPLES];

// Tone params
float toneFreq    = 7000.0f;
float tonePhase   = 0.0f;
float tonePhaseInc= 0.0f;
float phaseOffset = 0.0f;

float currentGain = 0.0f;
float targetGain  = 8000.0f;

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

void generateAndWriteBlock() {
  for (int i = 0; i < CHUNK_SAMPLES; i++) {
    float s = sinf(tonePhase + phaseOffset);
    tonePhase = wrap2pi(tonePhase + tonePhaseInc);

    float delta = targetGain - currentGain;
    if (fabsf(delta) > GAIN_SLEW_PER_SAMPLE) {
      currentGain += GAIN_SLEW_PER_SAMPLE * (delta > 0 ? 1.0f : -1.0f);
    } else {
      currentGain = targetGain;
    }

    int16_t val16 = (int16_t)(s * currentGain * PLAYBACK_ATTEN);
    outBuffer[i] = (int32_t)val16 << 16;  // left channel
  }

  size_t bw;
  i2s_write(I2S_NUM_0, (char *)outBuffer, sizeof(outBuffer), &bw, portMAX_DELAY);
}

void beep(float f, int ms) {
  float savedFreq  = toneFreq;
  float savedInc   = tonePhaseInc;
  float savedPhase = tonePhase;

  toneFreq = f;
  computePhaseIncrement();
  tonePhase = 0.0f;

  int totalSamples = (SAMPLE_RATE * ms) / 1000;
  while (totalSamples > 0) {
    int n = (totalSamples > CHUNK_SAMPLES ? CHUNK_SAMPLES : totalSamples);
    for (int i = 0; i < n; i++) {
      float s = sinf(tonePhase);
      tonePhase = wrap2pi(tonePhase + tonePhaseInc);
      int16_t val16 = (int16_t)(s * 12000.0f);
      outBuffer[i] = (int32_t)val16 << 16;
    }
    size_t bw;
    i2s_write(I2S_NUM_0, (char *)outBuffer, n * sizeof(int32_t), &bw, portMAX_DELAY);
    totalSamples -= n;
  }

  toneFreq    = savedFreq;
  tonePhaseInc= savedInc;
  tonePhase   = savedPhase;
}

void dumpSpectrumCSV(const char* tag) {
  size_t bytesRead;
  i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

  for (int i = 0; i < BUFFER_SIZE; i++) {
    vReal[i] = (double)sampleBuffer[i];
    vImag[i] = 0.0;
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  for (int i = 10; i < BUFFER_SIZE / 2; i += 2) {
    float f = (float)i * SAMPLE_RATE / BUFFER_SIZE;
    if (f < 2000.0f || f > 12000.0f) continue;
    Serial.printf("%s,%.0f,%.3f\n", tag, f, vReal[i]);
  }
}

float detectDominantFrequency(float &ampOut) {
  size_t bytesRead;
  i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

  for (int i = 0; i < BUFFER_SIZE; i++) {
    vReal[i] = (double)sampleBuffer[i];
    vImag[i] = 0.0;
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  int startBin = (int)(2000.0 * BUFFER_SIZE / SAMPLE_RATE);
  int stopBin  = (int)(12000.0 * BUFFER_SIZE / SAMPLE_RATE);

  int peakBin = startBin;
  double peakVal = vReal[startBin];

  for (int i = startBin; i < stopBin; i++) {
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

float measureAtFrequency(float f, int averages = 3) {
  float sum = 0.0f;
  int bin = (int)(f * BUFFER_SIZE / SAMPLE_RATE);
  for (int a = 0; a < averages; a++) {
    size_t bytesRead;
    i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

    for (int i = 0; i < BUFFER_SIZE; i++) {
      vReal[i] = (double)sampleBuffer[i];
      vImag[i] = 0.0;
    }
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    sum += vReal[bin];
  }
  return sum / (float)averages;
}

// -------------------- Calibration --------------------
void runCalibration() {
  calibrating = true;
  Serial.println("=== CALIBRATION START ===");

  // 1. stop speaker, get clean BEFORE
  outputEnabled = false;
  delay(120);
  dumpSpectrumCSV("BEFORE");

  // 2. clean FFT to find drill frequency
  float ampBefore = 0.0f;
  float detectedFreq = detectDominantFrequency(ampBefore);

  // set new tone to this
  toneFreq = detectedFreq;
  computePhaseIncrement();

  // derive gain from amplitude (simple rule)
  targetGain = (ampBefore / 2000.0f) * 1.5f;
  if (targetGain > 25000.0f) targetGain = 25000.0f;
  if (targetGain < 2000.0f)  targetGain = 2000.0f;

  Serial.printf("Detected Tone: %.2f Hz\n", detectedFreq);
  Serial.printf("Initial Amp: %.3f\n", ampBefore);
  Serial.printf("Target Gain: %.1f\n", targetGain);

  // 3. turn speaker back on for sweep
  outputEnabled = true;
  delay(50);

  float bestPh  = 0.0f;
  float bestAmp = 1e12;

  // coarse sweep
  for (float ph = -M_PI; ph <= M_PI; ph += PHASE_STEP_COARSE) {
    phaseOffset = ph;
    delay(DWELL_MS);
    float amp = measureAtFrequency(toneFreq, 3);
    Serial.printf("COARSE,%.2f,%.2f\n", ph, amp);
    if (amp < bestAmp) {
      bestAmp = amp;
      bestPh  = ph;
    }
  }

  // fine sweep around best
  for (float ph = bestPh - 0.5f; ph <= bestPh + 0.5f; ph += PHASE_STEP_FINE) {
    phaseOffset = ph;
    delay(DWELL_MS);
    float amp = measureAtFrequency(toneFreq, 3);
    Serial.printf("FINE,%.2f,%.2f\n", ph, amp);
    if (amp < bestAmp) {
      bestAmp = amp;
      bestPh  = ph;
    }
  }

  phaseOffset = bestPh;

  // 4. turn speaker off again to measure AFTER
  outputEnabled = false;
  delay(120);
  float ampAfter = measureAtFrequency(toneFreq, 5);
  float dropdB = 20.0f * log10f((ampBefore + 1e-9f) / (ampAfter + 1e-9f));

  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.printf("Frequency: %.2f Hz\n", toneFreq);
  Serial.printf("Initial Amp: %.4f\n", ampBefore);
  Serial.printf("Final Amp:   %.4f\n", ampAfter);
  Serial.printf("Reduction:   %.2f dB\n", dropdB);
  Serial.printf("Optimal Phase Offset: %.3f rad\n", phaseOffset);

  // dump AFTER spectrum for inspection
  dumpSpectrumCSV("AFTER");

  // 5. save to NVS
  prefs.begin("antiphonon", false);
  prefs.putFloat("freq", toneFreq);
  prefs.putFloat("phase", phaseOffset);
  prefs.putFloat("gain", targetGain);
  prefs.end();

  // 6. turn speaker back on with new params
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
  float savedF = prefs.getFloat("freq", 7000.0f);
  float savedP = prefs.getFloat("phase", 0.0f);
  float savedG = prefs.getFloat("gain", 8000.0f);
  prefs.end();

  toneFreq    = savedF;
  phaseOffset = savedP;
  targetGain  = savedG;
  computePhaseIncrement();

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
