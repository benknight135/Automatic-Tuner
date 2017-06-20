// Declare variables
#define LIN_OUT 1 // use the lin output function
#define FFT_N 128 // set to 128 bins in fft
#include <FFT.h> // FFT library

#define NOISE 200// threshold of noise
#define HARMONIC_THRESH 1000 //threshold of acceptable harmonic
#define LOW_LIMIT 5 // lowest allowed frequency bin

#define windSpeed 100 // number between 1-255 for controlling standard winding speed
double inTuneRange = 0.2; // range accepted as in tune (db from note)
double SNAP_MULTIPLIER = 0.07; //smoothing mutliplier
#define lowLimit 200 // noise limit for zero-crossing threshold

// Include libraries
#include <math.h> // maths library
#include <mtr.h> // DC motor library
#include <oLED.h> // oLED display library
#include <ledArray.h> // led array library

// Declare I/O Pins
#define audioIn A0
#define btnCycle 2
#define btnSelect 3
#define IN1 11
#define IN2 12
#define EN1 10
#define ledRedL A1
#define ledYellowL A2
#define ledGreen 4
#define ledYellowR 5
#define ledRedR 6

// create objects of classes
mtr motor1(EN1, IN1, IN2);
oLED oLED1(false);
ledArray leds(ledRedL, ledYellowL, ledGreen, ledYellowR, ledRedR);

// inialise global variables
char cNote = 'E';
int iNote = 2;
int iOctave = 2;
bool bSharp = false;
int charaSelect = 1;
bool charaDisplay = true;
int charaCycle = 0;
unsigned int prevTimeCycle;
unsigned int prevTimeSelect;
double lastGoodFreq = 0;
double lastGoodDist = 0;
int errorCount = 0;
int sucessCount = 0;
double globalDist2Target = 0;
int mode = 0;
int mtrDirection = true;
int mtrSpeed = 0;
double smoothFreq = 0;
int success = 0;
float FFTFreq = 0;
float prevFrequency = 0;

const char aNotes[7] = {
  'C', 'D', 'E', 'F', 'G', 'A', 'B'
};

const double noteFreqs[12] = { // freq at octave n = freq at root * (2^n) e.g. A4 = 27.5*(2^n) = 440
  16.35, 17.32, 18.35, 19.45, 20.60, 21.83, 23.12, 24.5, 25.96, 27.5, 29.14, 30.87
};

void setup()   {
  // start serial for error checking
  Serial.begin(115200);

  // define O/I
  pinMode(btnCycle, INPUT);
  pinMode(btnSelect, INPUT);
  pinMode(audioIn, INPUT);
  
  // attach interrupts to buttons
  attachInterrupt(digitalPinToInterrupt(btnCycle), btnCycle_pressed, RISING);
  attachInterrupt(digitalPinToInterrupt(btnSelect), btnSelect_pressed, RISING);

  // run startup sequence
  startup();
}

void loop() {
  if (mode == 0) { //mode selection menu
    if (digitalRead(btnSelect) == HIGH) { // automatic mode selected
      oLED1.drawMenu("   Select Mode", "  Mode: Automatic", "", "");
      delay(500);
      mode = 1; //note selection
    } else if (digitalRead(btnCycle) == HIGH) { // manual mode selected
      oLED1.drawMenu("   Select Mode", "", "Mode: Manual", "");
      delay(500);
      mode = 3; //manual mode
    } else {
      oLED1.drawMenu("   Select Mode", "  Red: Automatic", "Black: Manual", "");
    }
  } else if (mode == 1) { // note selection
    // flash note selection
    charaDisplay = !charaDisplay;
    oLED1.noteSelection(cNote, iOctave, bSharp, charaSelect, charaDisplay);
    delay(200); // letter flash speed
  } else if (mode == 2) { // automatic tuning
    // find and display distance to target note
    float freq = getFrequency();
  } else if (mode == 3) { // manual tuning
    if (digitalRead(btnCycle) == HIGH){ //motor forward
      oLED1.drawMenu("   Manual Mode", "  Red: Forward", "Black: Backward", "<");
      motor1.motorSpin(true, windSpeed);
    } else if (digitalRead(btnSelect) == HIGH) { //motor backward
      oLED1.drawMenu("   Manual Mode", "  Red: Forward", "Black: Backward", "                >");
      motor1.motorSpin(false, windSpeed);
    } else { // motor stop
      oLED1.drawMenu("   Manual Mode", "  Red: Forward", "Black: Backward", "");
    }
  }
}

void startup() {
  //startup sequence
  motor1.motorStop();
  oLED1.startup();
  leds.startup();
  leds.allOff();
}

double getFFT() {
  unsigned long startTime = micros(); //store current time for use in sample rate calculation
  for (int i = 0 ; i < FFT_N * 2 ; i += 2) { // save N samples
    int k = analogRead(audioIn);
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    fft_input[i] = k; // put real data into even bins
    fft_input[i + 1] = 0; // set odd bins to 0
    delayMicroseconds(810); // delay to keep sample rate around 1046Hz
  }
  unsigned long sampleTimeUS = (micros() - startTime); //time taken to take 128 samples
  double sampleTimeS = (double)sampleTimeUS / 1000000; //convert to seconds
  double SAMPLE_RATE = 1 / (sampleTimeS / FFT_N); // calcuate sample rate
  int BIN_SIZE = SAMPLE_RATE / FFT_N; // calcuate size of each bin

  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_lin(); // take the output of the fft

  double maxM = 0;
  int maxK = 0;
  // add lower frequency limit
  for (byte i = LOW_LIMIT ; i < (FFT_N / 2); i++) {
    if (fft_lin_out[i] > maxM) {
      maxK = i;
      maxM = fft_lin_out[i];
    }
  }
  double freq = 0;
  int K = maxK;
  // noise and harmonic detection
  if (maxM > NOISE) {
    freq = (double)(K) * SAMPLE_RATE / FFT_N;
    while (K > LOW_LIMIT) {
      double val = fft_lin_out[K];
      if (val > HARMONIC_THRESH) {
        freq = (double)(K) * SAMPLE_RATE / FFT_N;
        break;
      }
      K = K / 2;
    }
  }
  return (freq);
}


float getFrequency() {
  // reset variables
  int prevWaveData = 0; //previous wave data
  int newWaveData = 0; //new wave data
  double midpoint = 0; // midpoint threshold
  double maxVal = 700; // inital max amplitude value
  int midCount = 0; // count number of midpoints
  unsigned long totPeriod = 0; // total periods

  unsigned long newTime; // time at current midpoint
  unsigned long prevTime = micros(); // time at previous midpoint

  double frequency = 0;
  double freqGuess = getFFT();
  Serial.print("FFT: ");;
  Serial.println(freqGuess);

  if (freqGuess > 30 && freqGuess < 530) { //check FFT frequency is sensible value
    while (midCount < 20) { // repeat until 20 period measurements have been taken
      prevWaveData = newWaveData;//store previous value
      newWaveData = analogRead(audioIn); //record new value
      midpoint = maxVal - ((maxVal / 100) * 10); // midpoint is 10% of max value
      unsigned long newTime = micros();
      unsigned long period = newTime - prevTime; // calcuate period between crossing points
      if (period > 500000) { //stop function getting stuck when no signal is being read
        break;
      }
      if (prevWaveData < (midpoint) && newWaveData >= (midpoint)) { //if slope is increasing and crossing midpoint
        if (midCount != 0) { // only find period when more than one midpoint has been found
          totPeriod = totPeriod + period; // period to rolling total
        }
        prevTime = newTime; // new previous time is the midpoit just found
        midCount++; // add 1 to number of midpoints found
      } else {
        maxVal = maxVal - SNAP_MULTIPLIER; // decay maximum voltage
      }
      constrain(midpoint, lowLimit, 1024); //stop threshold fallling below noise level set by 'lowLimit'
	  if (newWaveData > maxVal){ //update maximum voltage level
		maxVal = newWaveData;
	  }
    }
    double avPeriodUS = ((double)totPeriod / (double)(midCount - 1)); // average period per midpoint in microseconds (midCount-1 as the first point is never recorded)
    double avPeriodS = avPeriodUS / 1000000; // average period per midpoint in seconds

    frequency = 1 / avPeriodS; // calcuate frequency from period average

    //divide by 2 untill less than double (detect harmonic)
    double freqDiff = frequency / freqGuess;
    while (freqDiff > 1.7) {
      Serial.print("harmonic: ");
      Serial.println(frequency);
      frequency = frequency / 2;
      freqDiff = frequency / freqGuess;
    }
  }
  Serial.print("frequency: ");
  Serial.println(frequency);
  double freqChange = 10 * log(abs(frequency - prevFrequency));
  if (freqChange < 3) { // only accept frequency if a close frequency is detected twice in a row
    //add frequency measurement to smoothing average
    float snap = snapCurve((abs(frequency - smoothFreq)) * SNAP_MULTIPLIER);
    smoothFreq += (frequency - smoothFreq) * snap;
	//find distance to note goal
    double freqDist = distanceToNote(cNote, bSharp, iOctave, smoothFreq);
    double freqToMatch = noteFreq(cNote, bSharp, iOctave);
	//display frequency measurement to user
    oLED1.tuner(smoothFreq, (float)freqDist);
    leds.tuner((float)freqDist, inTuneRange);
    if (frequency > 0) { //only move motor if frequency is read correctly
      bool curDirection = mtrDirection;
      Serial.println(curDirection);
      if (freqDist > inTuneRange) { // too high
        curDirection = !mtrDirection;
		motor1.motorSpin(curDirection, windSpeed);
      } else if (freqDist < -inTuneRange) { // too low
        curDirection = mtrDirection;
		motor1.motorSpin(curDirection, windSpeed);
      }
    }
  }
  prevFrequency = frequency;
  return (smoothFreq);
}

double snapCurve(double x) { //curve used for frequency smoothing
  double y = 1 / (x + 1);
  y = (1 - y) * 2;
  if (y > 1) {
    return 1;
  }
  return y;
};

double distanceToNote(char noteLetter, bool sharp, int octaveIdx, double frequency) { //distance from current frequency to goal note
  if (frequency == 0) {
    return (0);
  } else {
    double freqToMatch = noteFreq(noteLetter, sharp, octaveIdx);
    double dFreqDist = 10 * log(frequency / freqToMatch);
    dFreqDist = constrain(dFreqDist, -12, 12);
    return (dFreqDist);
  }
}

double noteFreq(char noteLetter, bool sharp, int octaveIdx) { //frequency of note
  int noteIdx;
  // map note in text to location in frequency array
  for (int i = 0; i < 7; i++) {
    if (aNotes[i] == noteLetter) {
      if (i < 3) {
        noteIdx = i * 2;
      } else {
        noteIdx = (i * 2) - 1;
      }
      if (sharp) {
        noteIdx = noteIdx + 1;
      }
      break;
    }
  }
  //f = base frequency * (2^octave)
  int octaveScale = power(2, octaveIdx);
  double freqToMatch = (double)noteFreqs[noteIdx] * octaveScale;
  return (freqToMatch);
}

int power(int base, int product) {// used in place of pow function as error was found when using the in build function
  int result = 1;
  if (product > 0) {
    for (unsigned int i = 0; i < product; i++)
    {
      result = result * base;
    }
  } else {
    for (unsigned int i = 0; i > product; i--)
    {
      result = result / base;
    }
  }
  return (result);
}

void btnCycle_pressed() { //interrupt button function
  //prevent button bounce
  int newTime = millis();
  if (newTime - prevTimeCycle > 250) {
    if (digitalRead(btnSelect) == HIGH) { //both buttons pressed
      mode = 0;
      charaSelect = 1;
    } else if (mode == 1) {
		// cycle through character
      switch (charaSelect) {
        case 1:
          charaCycle = iNote;
          charaCycle++;
          if (charaCycle > 6) {
            charaCycle = 0;
          }
          cNote = aNotes[charaCycle];
          iNote = charaCycle;
          break;
        case 2:
          charaCycle = iOctave;
          charaCycle++;
          if (charaCycle > 9) {
            charaCycle = 0;
          }
          iOctave = charaCycle;
          break;
        case 3:
          bSharp = !bSharp;
          break;
        default:
          // should never happen. error.
          break;
      }
      charaDisplay = false;
    } else if (mode == 2 || mode == 3) {
	  // pick motor direction
      mtrDirection = false;
    }
    prevTimeCycle = newTime;
  }
}

void btnSelect_pressed() { //interrupt button function
  //prevent button bounce
  int newTime = millis();
  if (newTime - prevTimeSelect > 250) {
    if (digitalRead(btnCycle) == HIGH) { //both buttons pressed
      mode = 0;
      charaSelect = 1;
    } else if (mode == 1) {
      //accept character and move onto next column
      charaCycle = 0;
      charaSelect++;
      if (charaSelect > 3) {
        charaSelect = 1;
        mode = 2;
      }
      charaDisplay = false;
    } else if (mode == 2 || mode == 3) {
	  // pick motor direction
      mtrDirection = true;
    }
    prevTimeSelect = newTime;
  }
}
