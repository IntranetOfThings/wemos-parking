// based on http://www.instructables.com/id/Ultrasonic-Garage-Parking-Assistant-with-Arduino/?ALLSTEPS

// use the NewPing library for accessing the sonar
#include <NewPing.h>
#include <Homie.h>

#define FW_NAME             "wemos-parking"
#define FW_VERSION          "0.0.1"

/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */

// maximum distance we want to ping for (in centimeters). 
// maximum sensor distance is rated at 400-500cm.
#define MAX_DISTANCE_CM     300
#define PING_DELAY_MS       100
#define PING_AVERAGE_COUNT  5

// hardware pins
#define ECHO_PIN            D1
#define TRIGGER_PIN         D2
#define RED_LED             D6
#define GREEN_LED           D7
#define BLUE_LED            D8

// state variables, for debugging via serial only
#define TIMED_OUT           1
#define NO_CAR              2
#define CAR_DETECTED        3
#define CAR_APPROACHING     4
#define CAR_CLOSE           5
#define CAR_PARKED          6
#define CAR_TOO_CLOSE       7

// parking thresholds for the various notifications
int distance    = 100;      // initialise to something sensible
int range       = 15;       // initialise to something sensible
int upper       = 0;             
int lower       = 0;
int approach    = 0;
int state       = 0;

// keep track of distances to identify when car is parked
int pingAverager[ PING_AVERAGE_COUNT ] = {};
int previous = 0;
int lastPublish = 0;
int count = 0;
int parked = -1;

HomieNode parkNode("park", "state");

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);

bool distanceHandler(String value) {
  distance = value.toInt();
  updateThresholds(); 
  return true;
}

bool rangeHandler(String value) {
  range = value.toInt();
  updateThresholds(); 
  return true;
}

void setupHandler() {  
  Serial.println("Initialising LEDs...");
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  Serial.println(" - red test");
  red();
  delay(2000);
  Serial.println(" - green test");
  green();
  delay(2000);
  Serial.println(" - blue test");
  blue();
  delay(2000);

  // initialise our ping averager
  for (int i = 0; i < PING_AVERAGE_COUNT; i++) {
    pingAverager[i] = 0;
  }

  // calculate our initial thresholds
  updateThresholds();
  
  Serial.println("Sensor active!");
}

void loopHandler() {
  // measure the distance (in cm) - defaults to MAX_DISTANCE_CM if no reading
  int current = pingDistance();

  // rate limit the distance pubs
  int currentPublish = ((int)(current / 10) * 10);
  if (currentPublish != lastPublish) {
    if (currentPublish == MAX_DISTANCE_CM) {
      Homie.setNodeProperty(parkNode, "current", "", false);
    } else {      
      Homie.setNodeProperty(parkNode, "current", String(currentPublish), false);
    }
    lastPublish = currentPublish;
  }
    
  // haven't reached upper threshold yet
  if (current > upper) { 
    // if car moves slow enough, it may never reset the count in above code, we reset again, just in case
    count = 0;

    // sensor becomes innacurate at large distances
    if (current >= MAX_DISTANCE_CM) {
      updateState(NO_CAR);
    } else if (current < MAX_DISTANCE_CM && current > (distance + 125)) {
      updateState(CAR_DETECTED);
    } else if (current <= (distance + 125) && current > approach) {
      updateState(CAR_APPROACHING);
    } else if (current <= approach && current > upper) {
      updateState(CAR_CLOSE);
    } 
  }

  // within upper/lower thresholds - in correct parking spot!
  if (current <= upper && current >= lower) {
    // distance fluctuates some resulting in false movement detection
    if (current >= (previous - 2) && current <= (previous + 2)) {
      // increase count if car has not moved
      count++;
    } else {
      // reset count if there is a significant change
      count = 0;
    }
   
    // if distance doesn't change for 250 cycles (30 sec) turn off LED -> power saving mode
    if (count >= 250) {
      updateState(TIMED_OUT);
      // don't increment forever
      count = 250;
    } else {
      updateState(CAR_PARKED);
    }
  }

  // closer than lower threshold
  if (current < lower) {
    // same situation for if car moves very slowly    
    count = 0;
    updateState(CAR_TOO_CLOSE);
  }

  // update distances
  previous = current;
}

// calculate our thresholds etc
void updateThresholds() {
  // update the various thresholds
  upper     = distance + (1 * range);
  lower     = distance - (1 * range);
  approach  = distance + (3 * range);

  // debugging
  Serial.print("Set parking distance: ");
  Serial.print(distance);
  Serial.print("cm; range +/-");
  Serial.print(range);
  Serial.print("cm (upper ");
  Serial.print(upper);
  Serial.print("cm, lower ");
  Serial.print(lower);
  Serial.print("cm, approach ");
  Serial.print(approach);
  Serial.println("cm)");
}

void updateState(int newState) {
  // always set the colour for the new state (regardless if the same or not)
  switch (newState) {
    case NO_CAR:          off();        break;
    case CAR_DETECTED:    green();      break;
    case CAR_APPROACHING: green();      break;
    case CAR_CLOSE:       flashGreen(); break;
    case CAR_PARKED:      red();        break;
    case CAR_TOO_CLOSE:   flashRed();   break;
    case TIMED_OUT:       off();        break;    
  }
  
  // only do the following if there is a change in state
  if (newState != state) {
    state = newState;
    
    String description = "Unknown state";
    switch (newState) {
      case NO_CAR:          description = "No car detected";          break;
      case CAR_DETECTED:    description = "Car detected";             break;
      case CAR_APPROACHING: description = "Car approaching";          break;
      case CAR_CLOSE:       description = "Getting close, slow down"; break;
      case CAR_PARKED:      description = "Car is parked";            break;
      case CAR_TOO_CLOSE:   description = "Too close, backup!";       break;
      case TIMED_OUT:       description = "Timed out, turning off";   break;    
    }

    // ignore timeout state changes - since we want to keep the last state
    if (state != TIMED_OUT) {
      // update the parked state and description
      Homie.setNodeProperty(parkNode, "state", String(state), true);  
      Homie.setNodeProperty(parkNode, "description", description, true);  

      // set the special "parked" flag
      if (state == CAR_CLOSE || state == CAR_PARKED || state == CAR_TOO_CLOSE) {
        Homie.setNodeProperty(parkNode, "parked", "true", true);
      } else {
        Homie.setNodeProperty(parkNode, "parked", "false", true);
      }
    }
    
    // debugging
    Serial.println(description);
  }
}

void off()         { setRGB(0, 0, 0); }
void red()         { setRGB(1, 0, 0); }
void green()       { setRGB(0, 1, 0); }
void blue()        { setRGB(0, 0, 1); }

void setRGB(int r, int g, int b)
{
  if (r) { digitalWrite(RED_LED,   LOW); } else { digitalWrite(RED_LED,   HIGH); }
  if (g) { digitalWrite(GREEN_LED, LOW); } else { digitalWrite(GREEN_LED, HIGH); }
  if (b) { digitalWrite(BLUE_LED,  LOW); } else { digitalWrite(BLUE_LED,  HIGH); }
}

void flashRed() {
  red();
  delay(100);
  off();
  delay(100);
}

void flashGreen() {
  green();
  delay(100);
  off();
  delay(100);
}

void flashBlue() {
  blue();
  delay(100);
  off();
  delay(100);
}

long pingDistance() {
  // wait between pings - 29ms should be the shortest delay between pings
  delay(PING_DELAY_MS);
  
  // send ping, get ping time in microseconds (uS)
  unsigned int uS = sonar.ping();
  long cm = uS / US_ROUNDTRIP_CM;
  // if no distance is read, set at max distance
  if (cm == 0) cm = MAX_DISTANCE_CM;
  
  // shift all values in our averager down one (and count for averaging)
  int total = 0;
  for (int i = 1; i < PING_AVERAGE_COUNT; i++) {
    pingAverager[i - 1] = pingAverager[i];
    total = total + pingAverager[i];
  }

  // add our latest reading to the end of the list and our sub-total
  pingAverager[PING_AVERAGE_COUNT - 1] = cm;
  total = total + cm;

  // calculate the average
  return total / PING_AVERAGE_COUNT;  
} 

void setup() {
  Homie.setFirmware(FW_NAME, FW_VERSION);

  parkNode.subscribe("distance", distanceHandler);
  parkNode.subscribe("range", rangeHandler);
  Homie.registerNode(parkNode);
  
  Homie.setSetupFunction(setupHandler);
  Homie.setLoopFunction(loopHandler);

  Homie.enableBuiltInLedIndicator(false);
  Homie.setup();
}

void loop() {
  Homie.loop();
}
