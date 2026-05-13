// ------------------------------------------------------------
// how this script works:
// channel 1 = erm 1 + solenoid 1
// channel 2 = erm 2 + solenoid 2
//
// each channel does this:
// 1) ramp up erm as a note-onset cue
// 2) hold erm at peak briefly
// 3) give time for the user to place finger on solenoid
// 4) turn on solenoid at low pwm so user can press it down
// 5) ramp solenoid upward to push user off
// 6) pause
//
// the two channels are randomly staggered so they overlap in time
//
// serial commands:
// spacebar = stop everything
// s = start/restart random sequence
// q = toggle solenoid 1 direction
// w = toggle solenoid 2 direction
// x = stop everything
// ------------------------------------------------------------

// motor driver pins
const int motor1PWM = 5;   // d5, pwm output for motor 1
const int motor1DIR = 8;   // d8, direction output for motor 1

const int motor2PWM = 6;   // d6, pwm output for motor 2
const int motor2DIR = 7;   // d7, direction output for motor 2

// erm pwm pins
const int ERMpwmPin1 = 3;  // d3, pwm
const int ERMpwmPin2 = 9;  // d9, pwm

// base timing values
const unsigned long base_ERM_ramp_time_ms = 100;
const unsigned long base_ERM_peak_hold_ms = 200;
const unsigned long base_finger_placement_time_ms = 1500;
const unsigned long base_solenoid_low_hold_time_ms = 500;
const unsigned long base_solenoid_ramp_time_ms = 900;
const unsigned long base_pause_after_solenoid_ms = 350;

// random timing ranges
const unsigned long random_start_delay_min_ms = 250;
const unsigned long random_start_delay_max_ms = 900;

const unsigned long random_restart_delay_min_ms = 400;
const unsigned long random_restart_delay_max_ms = 1200;

// duty cycle values
float ERM_max_duty_fraction = 0.60;
float solenoid_low_duty_fraction = 0.75;
float solenoid_peak_duty_fraction = 0.95;

// global run state
bool systemRunning = true;

// sequence states
enum ChannelState {
  CHANNEL_IDLE,
  ERM_RAMP,
  ERM_PEAK_HOLD,
  FINGER_PLACEMENT_WAIT,
  SOLENOID_LOW_HOLD,
  SOLENOID_RAMP_UP,
  PAUSE_AFTER_SOLENOID
};

// channel structure
struct HapticChannel {
  int ERMpin;
  int motorPWM;
  int motorDIR;
  bool motorDirection;

  ChannelState state;
  unsigned long stateStartTime;
  unsigned long nextStartTime;

  unsigned long ERM_ramp_time_ms;
  unsigned long ERM_peak_hold_ms;
  unsigned long finger_placement_time_ms;
  unsigned long solenoid_low_hold_time_ms;
  unsigned long solenoid_ramp_time_ms;
  unsigned long pause_after_solenoid_ms;

  float ERM_duty_fraction;
};

// channel 1: erm 1 + solenoid 1
HapticChannel channel1 = {
  ERMpwmPin1,
  motor1PWM,
  motor1DIR,
  LOW,
  CHANNEL_IDLE,
  0,
  0,
  base_ERM_ramp_time_ms,
  base_ERM_peak_hold_ms,
  base_finger_placement_time_ms,
  base_solenoid_low_hold_time_ms,
  base_solenoid_ramp_time_ms,
  base_pause_after_solenoid_ms,
  ERM_max_duty_fraction
};

// channel 2: erm 2 + solenoid 2
HapticChannel channel2 = {
  ERMpwmPin2,
  motor2PWM,
  motor2DIR,
  LOW,
  CHANNEL_IDLE,
  0,
  0,
  base_ERM_ramp_time_ms,
  base_ERM_peak_hold_ms,
  base_finger_placement_time_ms,
  base_solenoid_low_hold_time_ms,
  base_solenoid_ramp_time_ms,
  base_pause_after_solenoid_ms,
  ERM_max_duty_fraction
};

// ------------------------------------------------------------
// helper function: convert duty fraction to pwm value
// ------------------------------------------------------------
int dutyToPWM(float duty_fraction) {
  duty_fraction = constrain(duty_fraction, 0.0, 1.0);
  return (int)(255.0 * duty_fraction);
}

// ------------------------------------------------------------
// helper function: safely compare millis times
// ------------------------------------------------------------
bool timeReached(unsigned long targetTime) {
  return ((long)(millis() - targetTime) >= 0);
}

// ------------------------------------------------------------
// helper function: turn one channel off
// ------------------------------------------------------------
void turnChannelOff(HapticChannel &channel) {
  analogWrite(channel.ERMpin, 0);
  analogWrite(channel.motorPWM, 0);
}

// ------------------------------------------------------------
// helper function: turn everything off
// ------------------------------------------------------------
void turnEverythingOff() {
  turnChannelOff(channel1);
  turnChannelOff(channel2);
}

// ------------------------------------------------------------
// helper function: randomize one channel pattern
// ------------------------------------------------------------
void randomizeChannelPattern(HapticChannel &channel) {
  channel.ERM_ramp_time_ms = random(220, 501);
  channel.ERM_peak_hold_ms = random(30, 91);
  channel.finger_placement_time_ms = random(1000, 1801);
  channel.solenoid_low_hold_time_ms = random(500, 1001);
  channel.solenoid_ramp_time_ms = random(600, 1201);
  channel.pause_after_solenoid_ms = random(200, 701);

  // keep erm below 60 percent duty for 3 v equivalent from a 5 v pwm source
  channel.ERM_duty_fraction = random(45, 61) / 100.0;
}

// ------------------------------------------------------------
// helper function: schedule one channel to start later
// ------------------------------------------------------------
void scheduleChannel(HapticChannel &channel, unsigned long delay_ms) {
  channel.nextStartTime = millis() + delay_ms;
}

// ------------------------------------------------------------
// helper function: start one channel
// ------------------------------------------------------------
void startChannel(HapticChannel &channel) {
  turnChannelOff(channel);
  randomizeChannelPattern(channel);

  channel.state = ERM_RAMP;
  channel.stateStartTime = millis();

  Serial.print("starting channel on erm pin ");
  Serial.println(channel.ERMpin);
}

// ------------------------------------------------------------
// helper function: finish one channel
// ------------------------------------------------------------
void finishChannel(HapticChannel &channel) {
  turnChannelOff(channel);
  channel.state = CHANNEL_IDLE;

  unsigned long restartDelay = random(random_restart_delay_min_ms, random_restart_delay_max_ms + 1);
  scheduleChannel(channel, restartDelay);
}

// ------------------------------------------------------------
// helper function: update erm ramp for one channel
// ------------------------------------------------------------
void updateERMRamp(HapticChannel &channel, unsigned long elapsedTime) {
  float ramp_fraction = (float)elapsedTime / (float)channel.ERM_ramp_time_ms;
  ramp_fraction = constrain(ramp_fraction, 0.0, 1.0);

  // quadratic ramp: starts soft and becomes stronger near the cue time
  float shaped_ramp = ramp_fraction * ramp_fraction;

  float current_duty = channel.ERM_duty_fraction * shaped_ramp;
  analogWrite(channel.ERMpin, dutyToPWM(current_duty));
}

// ------------------------------------------------------------
// helper function: update solenoid ramp for one channel
// ------------------------------------------------------------
void updateSolenoidRamp(HapticChannel &channel, unsigned long elapsedTime) {
  float ramp_fraction = (float)elapsedTime / (float)channel.solenoid_ramp_time_ms;
  ramp_fraction = constrain(ramp_fraction, 0.0, 1.0);

  float current_duty = solenoid_low_duty_fraction +
                       ramp_fraction * (solenoid_peak_duty_fraction - solenoid_low_duty_fraction);

  digitalWrite(channel.motorDIR, channel.motorDirection);
  analogWrite(channel.motorPWM, dutyToPWM(current_duty));
}

// ------------------------------------------------------------
// helper function: change one channel state
// ------------------------------------------------------------
void changeChannelState(HapticChannel &channel, ChannelState newState) {
  channel.state = newState;
  channel.stateStartTime = millis();

  // only turn off this channel, not the other channel
  turnChannelOff(channel);

  if (channel.state == ERM_PEAK_HOLD) {
    analogWrite(channel.ERMpin, dutyToPWM(channel.ERM_duty_fraction));
  }

  else if (channel.state == SOLENOID_LOW_HOLD) {
    digitalWrite(channel.motorDIR, channel.motorDirection);
    analogWrite(channel.motorPWM, dutyToPWM(solenoid_low_duty_fraction));
  }

  else if (channel.state == SOLENOID_RAMP_UP) {
    digitalWrite(channel.motorDIR, channel.motorDirection);
    analogWrite(channel.motorPWM, dutyToPWM(solenoid_low_duty_fraction));
  }
}

// ------------------------------------------------------------
// helper function: update one channel
// ------------------------------------------------------------
void updateChannel(HapticChannel &channel) {
  if (!systemRunning) {
    turnChannelOff(channel);
    return;
  }

  if (channel.state == CHANNEL_IDLE) {
    if (timeReached(channel.nextStartTime)) {
      startChannel(channel);
    }
    return;
  }

  unsigned long elapsedTime = millis() - channel.stateStartTime;

  if (channel.state == ERM_RAMP) {
    updateERMRamp(channel, elapsedTime);

    if (elapsedTime >= channel.ERM_ramp_time_ms) {
      changeChannelState(channel, ERM_PEAK_HOLD);
    }
  }

  else if (channel.state == ERM_PEAK_HOLD && elapsedTime >= channel.ERM_peak_hold_ms) {
    changeChannelState(channel, FINGER_PLACEMENT_WAIT);
  }

  else if (channel.state == FINGER_PLACEMENT_WAIT && elapsedTime >= channel.finger_placement_time_ms) {
    changeChannelState(channel, SOLENOID_LOW_HOLD);
  }

  else if (channel.state == SOLENOID_LOW_HOLD && elapsedTime >= channel.solenoid_low_hold_time_ms) {
    changeChannelState(channel, SOLENOID_RAMP_UP);
  }

  else if (channel.state == SOLENOID_RAMP_UP) {
    updateSolenoidRamp(channel, elapsedTime);

    if (elapsedTime >= channel.solenoid_ramp_time_ms) {
      changeChannelState(channel, PAUSE_AFTER_SOLENOID);
    }
  }

  else if (channel.state == PAUSE_AFTER_SOLENOID && elapsedTime >= channel.pause_after_solenoid_ms) {
    finishChannel(channel);
  }
}

// ------------------------------------------------------------
// helper function: start randomized overlapping pattern
// ------------------------------------------------------------
void restartRandomPattern() {
  systemRunning = true;
  turnEverythingOff();

  channel1.state = CHANNEL_IDLE;
  channel2.state = CHANNEL_IDLE;

  // channel 1 starts immediately
  scheduleChannel(channel1, 0);

  // channel 2 starts shortly after channel 1, creating overlap
  unsigned long offsetDelay = random(random_start_delay_min_ms, random_start_delay_max_ms + 1);
  scheduleChannel(channel2, offsetDelay);

  Serial.println("random overlapping pattern restarted");
}

// ------------------------------------------------------------
// function to read all available serial commands
// ------------------------------------------------------------
void readSerialCommand() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (command == ' ') {
      systemRunning = false;
      turnEverythingOff();
      Serial.println("stopped by spacebar");
    }

    else if (command == 'x') {
      systemRunning = false;
      turnEverythingOff();
      Serial.println("all outputs off");
    }

    else if (command == 's') {
      restartRandomPattern();
    }

    else if (command == 'q') {
      channel1.motorDirection = !channel1.motorDirection;
      digitalWrite(channel1.motorDIR, channel1.motorDirection);
      Serial.print("channel 1 direction = ");
      Serial.println(channel1.motorDirection);
    }

    else if (command == 'w') {
      channel2.motorDirection = !channel2.motorDirection;
      digitalWrite(channel2.motorDIR, channel2.motorDirection);
      Serial.print("channel 2 direction = ");
      Serial.println(channel2.motorDirection);
    }

    else {
      // ignore newline, carriage return, tabs, or unknown characters
    }
  }
}

// ------------------------------------------------------------
// setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(channel1.ERMpin, OUTPUT);
  pinMode(channel1.motorPWM, OUTPUT);
  pinMode(channel1.motorDIR, OUTPUT);

  pinMode(channel2.ERMpin, OUTPUT);
  pinMode(channel2.motorPWM, OUTPUT);
  pinMode(channel2.motorDIR, OUTPUT);

  digitalWrite(channel1.motorDIR, channel1.motorDirection);
  digitalWrite(channel2.motorDIR, channel2.motorDirection);

  turnEverythingOff();

  randomSeed(analogRead(A0));

  restartRandomPattern();

  Serial.println("hapkit random overlapping erm + solenoid system ready");
  Serial.println("spacebar = stop, s = restart random pattern, q/w = toggle directions");
}

// ------------------------------------------------------------
// main loop
// ------------------------------------------------------------
void loop() {
  readSerialCommand();

  updateChannel(channel1);
  updateChannel(channel2);
}