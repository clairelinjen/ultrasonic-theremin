/*      
Built from YummyDSP example code for ESP32-A1S
and ultrasonic sensor example code from https://www.intorobotics.com/object-detection-hc-sr04-arduino-millis/
*/


#include <YummyDSP.h>
#include <AudioDriver.h>
#include <Codecs/AC101/AC101.h>

#define I2S_BCK_PIN                 27
#define I2S_LRCLK_PIN               26
#define I2S_DOUT_PIN                25
#define I2S_DIN_PIN             	35

#define GPIO_PA_EN                  GPIO_NUM_21
#define GPIO_SEL_PA_EN              GPIO_SEL_21

static AC101 i2sCodec;
YummyDSP dsp;
WaveSynth synth;
FilterNode lp;
FilterNode hp;

// I2S
const int fs = 96000;
const int channelCount = 2;

static uint8_t volume = 0;
const uint8_t volume_step = 1;

void audioTask(void *);

#define trigPitch 22 // define trig for pitch sensor
#define echoPitch 21 // define echo for pitch sensor
#define trigVol 19 // define trig for vol sensor
#define echoVol 23 // define echo for vol sensor
#define MAX_DISTANCE 700 // Maximum sensor distance is rated at 400-500cm. //timeOut= 2*MAX_DISTANCE /100 /340 *1000000 = MAX_DISTANCE*58.8
float timeOut = MAX_DISTANCE * 60;
int soundVelocity = 340; // define sound speed=340m/s

int arraySize = 5;
float maxFreq = 1046.5;
float lastPitch = 0.0;
float lastVol = 0.0;
float lastPitches[5];
float lastVols[5];
int pitchIndex = 0;
int volIndex = 0;

int increment(int i){
  i += 1;
  return i % arraySize;
}

float avgPitchRead(float val, float last, int index){
  float avg = 0.0;
  for (int i=0; i<arraySize; i++){
    avg += lastPitches[i];
  }
  avg += val;
  avg = avg/(arraySize+1);
  avg = (last+last+avg)/3;
  lastPitches[index] = avg;
  return avg;
}

float avgVolRead(int val, int last, int index){
  int avg = 0;
  for (int i=0; i<arraySize; i++){
    avg += lastVols[i];
  }
  avg += val;
  avg = avg/(arraySize+1);
  avg = (last+last+avg)/3;
  lastVols[index] = avg;
  return avg;
}

void updatePitch(float val)
{
  val = val/3;
  val = constrain(val,0,maxFreq);
  float newPitch = avgPitchRead(val, lastPitch, pitchIndex);
  lastPitch = newPitch;
  synth.note(newPitch);
  pitchIndex = increment(pitchIndex);
}

void updateVol(int val)
{
  val = constrain(val, 300, 1600);
  int newVolRead = avgVolRead(val, lastVol, volIndex);
  lastVol = newVolRead;
  int newVol = map(newVolRead, 0, 1600, 0, 20);
  uint8_t v = newVol;
  if (v != volume){
    i2sCodec.SetVolumeSpeaker(v);
    i2sCodec.SetVolumeHeadphone(v);
  }
  volIndex = increment(volIndex);  
}


void setup()
{
  Serial.begin(115200);

  Serial.printf("Connect to AC101 codec... ");

  // setup audio codec
  i2sCodec.setup(fs, channelCount, I2S_BCK_PIN, I2S_LRCLK_PIN, I2S_DOUT_PIN, I2S_DIN_PIN, GPIO_PA_EN);

  i2sCodec.SetVolumeSpeaker(volume);
  i2sCodec.SetVolumeHeadphone(volume);
  //  ac.DumpRegisters();

  // Enable amplifier
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  // setup audio lib
  dsp.begin(fs);

  synth.begin(fs);
  synth.noteOff();
  synth.setWaveform(SAW);
  synth.setGlide(1);
  synth.setAttack(50);
  synth.setSustain(0.6);

  // setup some filter nodes
  lp.begin(fs, channelCount);
  lp.setupFilter(FilterNode::LPF, 5000, 1.7);

  hp.begin(fs, channelCount);
  hp.setupFilter(FilterNode::HPF, 20, 1.0);

  // add nodes to audio processing tree
  // Synth => hp => lp => I2S out
  // dsp.addNode(&hp);
  dsp.addNode(&lp);


  // run audio in dedicated task on cpu core 1
  xTaskCreatePinnedToCore(audioTask, "audioTask", 10000, NULL, 10, NULL, 1);
  // run control task on another cpu  core with lower priority
  Serial.print("\nSetup done ");

  pinMode(trigPitch,OUTPUT);// set trigPin to output mode
  pinMode(trigVol,OUTPUT);
  pinMode(echoPitch,INPUT); // set echoPin to input mode
  pinMode(echoVol,INPUT);

  for (int i=0; i<arraySize; i++){
    lastPitches[i] = 0.0;
    lastVols[i] = 0;
  }

}

void audioTask(void *) {

  Serial.print("\nAudio task");

  float sample = 0;

  while (true) {

    for (int i = 0; i < AudioDriver::BufferSize; i++) {

      float sampleMono = synth.getSample();

      for (int ch = 0; ch < channelCount; ch++) {

        // upmix to stereo
        sample = sampleMono;

        sample = dsp.process(sample, ch);

        i2sCodec.writeSample(sample, i, ch);
      }
    }

    i2sCodec.writeBlock();
  }
  vTaskDelete(NULL);
}

unsigned long timeStart = 0;
int TIMER_TRIGGER_HIGH = 10;
int TIMER_LOW_HIGH = 2;
 
float timeDuration, distance;
 
/*The states of an ultrasonic sensor*/
enum SensorStates {
  TRIG_LOW,
  TRIG_HIGH,
  ECHO_HIGH
};
 
SensorStates _pitchSensorState = TRIG_LOW;
SensorStates _volSensorState = TRIG_LOW;
 
void startTimer() {
  timeStart = millis();
}
 
bool isTimerReady(int mSec) {
  return (millis() - timeStart) < mSec;
}

bool isPitch = true;
void loop() {
  delay(1.2);
  if (isPitch){
    /*Switch between the ultrasonic sensor states*/
  switch (_pitchSensorState) {
    /* Start with LOW pulse to ensure a clean HIGH pulse*/
    case TRIG_LOW: {
        digitalWrite(trigPitch, LOW);
        startTimer();
        if (isTimerReady(TIMER_LOW_HIGH)) {
          _pitchSensorState = TRIG_HIGH;
        }
      } break;
      
    /*Triggered a HIGH pulse of 10 microseconds*/
    case TRIG_HIGH: {
        digitalWrite(trigPitch, HIGH);
        startTimer();
        if (isTimerReady(TIMER_TRIGGER_HIGH)) {
          _pitchSensorState = ECHO_HIGH;
        }
      } break;
 
    /*Measures the time that ping took to return to the receiver.*/
    case ECHO_HIGH: {
        digitalWrite(trigPitch, LOW);
        timeDuration = pulseIn(echoPitch, HIGH);
        /*
           distance = time * speed of sound
           speed of sound is 340 m/s => 0.034 cm/us
        */
        updatePitch(timeDuration);
        //Serial.print("pitch sensor ");
        //Serial.println(timeDuration);
        _pitchSensorState = TRIG_LOW;
      } break;
      
  }//end switch
    isPitch = false;
  }
  else{
    switch (_volSensorState) {
    /* Start with LOW pulse to ensure a clean HIGH pulse*/
    case TRIG_LOW: {
        digitalWrite(trigVol, LOW);
        startTimer();
        if (isTimerReady(TIMER_LOW_HIGH)) {
          _volSensorState = TRIG_HIGH;
        }
      } break;
      
    /*Triggered a HIGH pulse of 10 microseconds*/
    case TRIG_HIGH: {
        digitalWrite(trigVol, HIGH);
        startTimer();
        if (isTimerReady(TIMER_TRIGGER_HIGH)) {
          _volSensorState = ECHO_HIGH;
        }
      } break;
 
    /*Measures the time that ping took to return to the receiver.*/
    case ECHO_HIGH: {
        digitalWrite(trigVol, LOW);
        timeDuration = pulseIn(echoVol, HIGH);
        /*
           distance = time * speed of sound
           speed of sound is 340 m/s => 0.034 cm/us
        */
        updateVol(timeDuration);
        //Serial.print("vol sensor ");
        _volSensorState = TRIG_LOW;
      } break;
      
  }//end switch
  isPitch = true;
  }
   
}//end loop
