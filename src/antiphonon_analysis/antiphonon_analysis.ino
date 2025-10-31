#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>

// -------------------- Config --------------------
#define BCLK_PIN    26
#define LRCK_PIN    25
#define SPK_DIN     22
#define MIC_DOUT    33
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 2048
#define CAL_BUTTON  15

#define GAIN_SLEW_PER_SAMPLE 0.0005f
#define PHASE_STEP_COARSE  (M_PI/18.0)   // 10°
#define PHASE_STEP_FINE    (M_PI/180.0)  // 1°
#define DWELL_MS           300
#define DONE_BEEP_FREQ     400.0f
#define DONE_BEEP_MS       300
#define CHUNK_SAMPLES      1024

#define PLAYBACK_ATTEN     0.70f

// FFT
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, SAMPLE_RATE);

// Buffers
int32_t sampleBuffer[BUFFER_SIZE];
int32_t outBuffer[CHUNK_SAMPLES];

// Tone params
float toneFreq = 7000;
float tonePhase = 0;
float tonePhaseInc = 0;
float phaseOffset = 0;

float currentGain = 0.0f;
float targetGain  = 8000.0f;

volatile bool calibrating   = false;
volatile bool outputEnabled = true;

// -------------------- I2S Config --------------------
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
  for (int i=0;i<CHUNK_SAMPLES;i++) {
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
  i2s_write(I2S_NUM_0,(char*)outBuffer,sizeof(outBuffer),&bw,portMAX_DELAY);
}

void beep(float f,int ms){
  float savedFreq = toneFreq;
  float savedInc  = tonePhaseInc;
  float savedPhase = tonePhase;

  toneFreq = f; 
  computePhaseIncrement(); 
  tonePhase = 0;

  int totalSamples = (SAMPLE_RATE * ms) / 1000;
  while(totalSamples > 0){
    int n = (totalSamples > CHUNK_SAMPLES ? CHUNK_SAMPLES : totalSamples);
    for (int i=0;i<n;i++) {
      float s = sinf(tonePhase);
      tonePhase = wrap2pi(tonePhase + tonePhaseInc);
      int16_t val16 = (int16_t)(s * 12000.0f);
      outBuffer[i] = (int32_t)val16 << 16;
    }
    size_t bw;
    i2s_write(I2S_NUM_0,(char*)outBuffer,n*sizeof(int32_t),&bw,portMAX_DELAY);
    totalSamples -= n;
  }

  toneFreq = savedFreq;
  tonePhaseInc = savedInc;
  tonePhase = savedPhase;
}

// -------------------- Measurement --------------------
float measureFFTAmplitude(float f, int averages=3) {
  float sum=0;
  for (int a=0;a<averages;a++) {
    size_t bytesRead;
    i2s_read(I2S_NUM_0,(char*)sampleBuffer,sizeof(sampleBuffer),&bytesRead,portMAX_DELAY);
    for (int i=0;i<BUFFER_SIZE;i++) {
      vReal[i] = (double)sampleBuffer[i];
      vImag[i] = 0.0;
    }
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    int bin = (int)(f * BUFFER_SIZE / SAMPLE_RATE);
    sum += vReal[bin];
  }
  return sum / averages;
}

void dumpSpectrumCSV(const char* tag) {
  size_t bytesRead;
  i2s_read(I2S_NUM_0,(char*)sampleBuffer,sizeof(sampleBuffer),&bytesRead,portMAX_DELAY);
  for (int i=0;i<BUFFER_SIZE;i++) {
    vReal[i] = (double)sampleBuffer[i];
    vImag[i] = 0.0;
  }
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  for (int i=10;i<BUFFER_SIZE/2;i+=2) {  // decimate
    float f = (float)i * SAMPLE_RATE / BUFFER_SIZE;
    if (f < 2000 || f > 12000) continue;
    Serial.printf("%s,%.0f,%.2f\n", tag, f, vReal[i]);
  }
}

// -------------------- Calibration --------------------
void calibrate() {
  calibrating = true;

  outputEnabled = false;
  delay(100);              
  dumpSpectrumCSV("BEFORE");
  outputEnabled = true;

  // --- FFT SEED ---
  size_t bytesRead;
  i2s_read(I2S_NUM_0,(char*)sampleBuffer,sizeof(sampleBuffer),&bytesRead,portMAX_DELAY);
  for (int i=0;i<BUFFER_SIZE;i++) {
    vReal[i] = (double)sampleBuffer[i];
    vImag[i] = 0.0;
  }
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  int startBin = (int)(2000.0 * BUFFER_SIZE / SAMPLE_RATE);
  int stopBin  = (int)(12000.0 * BUFFER_SIZE / SAMPLE_RATE);
  int peak = startBin; double pv = vReal[startBin];
  for(int i=startBin; i<stopBin; i++) {
    if(vReal[i] > pv) { pv=vReal[i]; peak=i; }
  }
  toneFreq = (float)peak * SAMPLE_RATE / BUFFER_SIZE;
  if (toneFreq < 2000 || toneFreq > 12000) toneFreq = 7000;
  computePhaseIncrement();

  float ampBefore = vReal[peak];
  Serial.printf("INFO,SeedFreq,%.1f,InitialAmp,%.2f\n", toneFreq, ampBefore);

  // --- Gain match + boost ---
  targetGain = (ampBefore / 2000.0f) * 1.5f;
  if (targetGain > 25000.0f) targetGain = 25000.0f;
  if (targetGain < 2000.0f)  targetGain = 2000.0f;
  Serial.printf("INFO,TargetGain,%.1f\n", targetGain);

  // --- Phase sweep ---
  float bestPh = 0;
  float bestAmp = 1e12;

  for (float ph=-M_PI; ph<=M_PI; ph+=PHASE_STEP_COARSE) {
    phaseOffset = ph;
    delay(DWELL_MS);
    float amp = measureFFTAmplitude(toneFreq, 3);
    Serial.printf("SWEEP,%.2f,%.2f\n", ph, amp);
    if (amp < bestAmp) { bestAmp = amp; bestPh = ph; }
  }

  for (float ph=bestPh-0.5; ph<=bestPh+0.5; ph+=PHASE_STEP_FINE) {
    phaseOffset = ph;
    delay(DWELL_MS);
    float amp = measureFFTAmplitude(toneFreq, 3);
    Serial.printf("SWEEP,%.2f,%.2f\n", ph, amp);
    if (amp < bestAmp) { bestAmp = amp; bestPh = ph; }
  }
  phaseOffset = bestPh;

  // --- Final re-measure ---
  float ampAfter = measureFFTAmplitude(toneFreq, 5);
  float ampDrop = 20*log10f((ampBefore+1e-9)/(ampAfter+1e-9));
  Serial.printf("RESULT,Before,%.2f\n", ampBefore);
  Serial.printf("RESULT,After,%.2f\n",  ampAfter);
  Serial.printf("RESULT,Drop_dB,%.2f\n", ampDrop);

  // --- Spectrum AFTER (pause output again) ---
  outputEnabled = false;
  delay(100);
  dumpSpectrumCSV("AFTER");
  outputEnabled = true;

  beep(DONE_BEEP_FREQ,DONE_BEEP_MS);
  calibrating = false;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  pinMode(CAL_BUTTON, INPUT_PULLUP);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// -------------------- Loop --------------------
void loop() {
  if (outputEnabled) {
    generateAndWriteBlock();
  }

  if (!calibrating && digitalRead(CAL_BUTTON) == LOW) {
    calibrate();
    delay(500);
  }
}
