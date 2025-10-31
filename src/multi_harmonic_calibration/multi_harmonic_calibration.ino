#include <driver/i2s.h>
#include <Preferences.h>
#include <arduinoFFT.h>
#include <math.h>

// -------------------- Config --------------------
#define USE_MULTI_TONE 0 
#define MAX_TONES (USE_MULTI_TONE ? 3 : 1)

#define BCLK_PIN    26
#define LRCK_PIN    25
#define SPK_DIN     22 
#define MIC_DOUT    33
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 2048
#define CAL_BUTTON  15

#define TARGET_GAIN 0.2f
#define GAIN_SLEW_PER_SAMPLE 0.0005f
#define MIN_SNR_DB 6.0f

Preferences prefs;

// FFT arrays
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, SAMPLE_RATE);

// Buffers
int32_t sampleBuffer[BUFFER_SIZE];   
int32_t outBuffer[64];              

// Tone parameters
float toneFreq[MAX_TONES];
float toneAmp[MAX_TONES];
float tonePhase[MAX_TONES];
float tonePhaseInc[MAX_TONES];

float currentGain = 0.0f;

// -------------------- I2S Config --------------------
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,   
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,    
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = 0,
  .dma_buf_count = 4,
  .dma_buf_len = 64,
  .use_apll = false,
  .tx_desc_auto_clear = true,
  .fixed_mclk = 0
};

i2s_pin_config_t pin_config = {
  .bck_io_num = BCLK_PIN,
  .ws_io_num = LRCK_PIN,
  .data_out_num = SPK_DIN,
  .data_in_num = MIC_DOUT
};

// -------------------- Helpers --------------------
float wrap2pi(float x) {
  if (x >= 2.0f * (float)M_PI) x -= 2.0f * (float)M_PI;
  else if (x < 0.0f) x += 2.0f * (float)M_PI;
  return x;
}

void computePhaseIncrements() {
  for (int h = 0; h < MAX_TONES; ++h) {
    tonePhaseInc[h] = (2.0f * (float)M_PI * toneFreq[h]) / SAMPLE_RATE;
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  pinMode(CAL_BUTTON, INPUT_PULLUP);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  prefs.begin("ancProfile", false);

  if (prefs.isKey("freq0")) {
    Serial.println("Loaded tones from flash:");
    for (int h = 0; h < MAX_TONES; h++) {
      toneFreq[h]  = prefs.getFloat(("freq" + String(h)).c_str(), 0);
      toneAmp[h]   = prefs.getFloat(("amp" + String(h)).c_str(), 0);
      tonePhase[h] = prefs.getFloat(("phase" + String(h)).c_str(), 0);
      Serial.printf("T%d: f=%.2f Hz, amp=%.3f, phase=%.3f rad\n",
                    h, toneFreq[h], toneAmp[h], tonePhase[h]);
    }
    computePhaseIncrements();
  } else {
    Serial.println("No tone data found. Please calibrate.");
  }
}

// -------------------- Loop --------------------
void loop() {
  if (digitalRead(CAL_BUTTON) == LOW) {
    Serial.println("Calibration button pressed — clearing flash...");
    prefs.clear();
    for (int i = 3; i > 0; i--) {
      Serial.printf("Calibrating in %d...\n", i);
      delay(1000);
    }
    calibrate();
    delay(500);
  }
  runToneOutput();
}

// -------------------- Calibration --------------------
void calibrate() {
  size_t bytesRead;
  Serial.println("Recording samples...");
  i2s_read(I2S_NUM_0, (char*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

  int samplesRead = bytesRead / sizeof(int32_t);
  if (samplesRead < BUFFER_SIZE) {
    Serial.println("Warning: fewer samples read than expected.");
  }

  // Copy to FFT arrays
  for (int i = 0; i < BUFFER_SIZE; i++) {
    vReal[i] = (double)sampleBuffer[i];
    vImag[i] = 0.0;
  }

  // Remove DC offset
  double mean = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) mean += vReal[i];
  mean /= BUFFER_SIZE;
  for (int i = 0; i < BUFFER_SIZE; i++) vReal[i] -= mean;

  FFT.windowing(FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // Find peaks
  int picked = 0;
  bool binUsed[BUFFER_SIZE / 2] = {false};

  while (picked < MAX_TONES) {
    int peakIndex = -1;
    double peakValue = 0;
    for (int i = 1; i < BUFFER_SIZE / 2; i++) {
      if (!binUsed[i] && vReal[i] > peakValue) {
        peakValue = vReal[i];
        peakIndex = i;
      }
    }
    if (peakIndex < 0) break;

    float freq = ((float)peakIndex * SAMPLE_RATE) / BUFFER_SIZE;
    toneFreq[picked]  = freq;
    toneAmp[picked]   = 1.0f;   // normalize later
    tonePhase[picked] = 0.0f;

    prefs.putFloat(("freq" + String(picked)).c_str(), toneFreq[picked]);
    prefs.putFloat(("amp" + String(picked)).c_str(), toneAmp[picked]);
    prefs.putFloat(("phase" + String(picked)).c_str(), tonePhase[picked]);

    Serial.printf("T%d: f=%.2f Hz (bin %d)\n", picked, toneFreq[picked], peakIndex);

    for (int j = -5; j <= 5; j++) {
      int idx = peakIndex + j;
      if (idx >= 0 && idx < BUFFER_SIZE / 2) binUsed[idx] = true;
    }
    picked++;
  }

  computePhaseIncrements();

  Serial.println("Calibration done.");
}

// -------------------- Playback --------------------
void runToneOutput() {
  size_t bytesWritten;
  for (int i = 0; i < 64; i++) {
    float delta = TARGET_GAIN - currentGain;
    if (fabsf(delta) > GAIN_SLEW_PER_SAMPLE)
      currentGain += GAIN_SLEW_PER_SAMPLE * (delta > 0 ? 1.0f : -1.0f);
    else
      currentGain = TARGET_GAIN;

    float sample = 0.0f;
    for (int h = 0; h < MAX_TONES; h++) {
      if (toneFreq[h] > 0 && toneAmp[h] > 0) {
        sample += toneAmp[h] * sinf(tonePhase[h]);
        tonePhase[h] = wrap2pi(tonePhase[h] + tonePhaseInc[h]);
      }
    }

    int16_t val16 = (int16_t)(sample * currentGain * 15000.0f);
    outBuffer[i] = (int32_t)val16 << 16;
  }
  i2s_write(I2S_NUM_0, (char*)outBuffer, sizeof(outBuffer), &bytesWritten, portMAX_DELAY);
}
