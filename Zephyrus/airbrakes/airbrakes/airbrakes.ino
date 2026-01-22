/// C Arduino Airbrakes Controller Code
// MARC D. NICHITIU írta
// MIT RT SIMULATIONS JAN 2026


#include <Arduino.h>
#include <math.h>
#include <stddef.h>

/* ------------------ Compile-time constants ------------------ */
#define AIRBRAKES_N_MEASUREMENTS         13
#define AIRBRAKES_MEASUREMENT_FREQ_HZ     5
#define AIRBRAKES_SIMULATION_T_APOG      34.0f
#define DEBUG_AIRBRAKES_ON               0
#define LOOP_FREQ                        100

/* ------------------ Measurements types ------------------ */
typedef struct {
  float velocityMeasurement;
  float timeStamp;
} AirbrakesVelocityMeasurement;

typedef struct {
  float accelerationMeasurement;
  float timeStamp;
} AirbrakesAccelerationMeasurement;

/* Constructors */
AirbrakesVelocityMeasurement AirbrakesVelocityMeasurement_init(float ts, float vm) {
  AirbrakesVelocityMeasurement m;
  m.timeStamp = ts;
  m.velocityMeasurement = vm;
  return m;
}

AirbrakesAccelerationMeasurement AirbrakesAccelerationMeasurement_init(float ts, float am) {
  AirbrakesAccelerationMeasurement m;
  m.timeStamp = ts;
  m.accelerationMeasurement = am;
  return m;
}

/* ------------------ RocketStatus placeholder ------------------ */
typedef struct {
  float altitude;        // m
  float vel_z;           // m/s
  float accel_z;         // m/s^2
  bool  apogeeReached;
} RocketStatus;

/* ------------------ Controller state ------------------ */
typedef enum {
  DISABLED = 0,
  PREP,
  PREPROCESS,
  WAIT_FOR_START,
  CONTROLLING_RAMP,
  CONTROLLING_PLATEAU,
  DONE,
  INFEASIBLE
} AirbrakesControllerState;

/* ------------------ Globals ------------------ */
AirbrakesControllerState state = DISABLED;

const float g = 9.81f;

float mass        = 51.75379038f;
float rho         = 0.82826203f;
float airbrakesCd = 1.28f;
float a_max       = 0.0066f;
float fudge_factor   = 3.2f;
float fudge_factor_2 = 3.5f;

float EARLIEST_START_AIRBRAKES_PREP_TIME = 4.0f;
float START_AIRBRAKES_PREP_VEL           = 400.0f;
float START_AIRBRAKES_PREPROC_TIME       = 12.5f;
float AIRBRAKES_TIME_DELAY               = 1.0f;
float AIRBRAKES_T_APOG_FUDGEDIFF         = 1.5f;

int   roundToHowMuch = 100;

float t_apog   = 35.5f;
float coeffA   = -0.0154397511f;
float coeffB   = -0.3379534959f;

float alt0          = 0.0f;
float predictedAlt  = 0.0f;
float desiredDeltaX = 0.0f;

float airbrakesCtrlStartTime = 1e10f;
float A0_req = 0.0f;

/* live rocket values !!! REPLACE IN LOOP W TELEMETRY */
float currentRocketVel   = 0.0f;
float currentRocketAccel = 0.0f;
bool  apogeeReached      = false;

float lastTimeStamp      = 0;

/* data collection */
AirbrakesAccelerationMeasurement accelData[AIRBRAKES_N_MEASUREMENTS];
AirbrakesVelocityMeasurement     velData[AIRBRAKES_N_MEASUREMENTS];

/* counters */
int datIndex = 0;
int counter  = 0;

/* ------------------ Helpers (declare before use) ------------------ */

float maxf(float a, float b) { return (a > b) ? a : b; }

/* Fast integer-power helpers */
float p4(float x){ float x2=x*x; return x2*x2; }
float p5(float x){ return p4(x)*x; }
float p6(float x){ float x3=x*x*x; return x3*x3; }
float p7(float x){ return p6(x)*x; }
float p8(float x){ float x4=p4(x); return x4*x4; }
float p9(float x){ return p8(x)*x; }
float p10(float x){ float x5=p5(x); return x5*x5; }

float pow5f_fast(float x){ return p5(x); }
float pow10f_fast(float x){ return p10(x); }

/* Avoid Arduino macro collision with sq() by using different names */
float sqf_local(float x) { return x * x; }
float cubef_local(float x) { return x * x * x; }
float pow4f_local(float x) { float x2 = x*x; return x2*x2; }

bool inverse2x2Matrix(const float A[2][2], float Ainv[2][2]) {
  float a = A[0][0], b = A[0][1];
  float c = A[1][0], d = A[1][1];
  float det = a*d - b*c;

  if (fabsf(det) <= 1e-6f) {
    Ainv[0][0]=0.0f; Ainv[0][1]=0.0f;
    Ainv[1][0]=0.0f; Ainv[1][1]=0.0f;
    return false;
  }

  float f = 1.0f / det;
  Ainv[0][0] =  f*d;
  Ainv[0][1] = -f*c;
  Ainv[1][0] = -f*b;
  Ainv[1][1] =  f*a;
  return true;
}

/* Flight time in seconds */
float getFlightTime() {
  return millis() * 0.001f;  // FIXED (seconds)
}

/* actuator command */
void setAirbrakesServo(float deployedFraction) {
  if (deployedFraction < 0.0f) deployedFraction = 0.0f;
  if (deployedFraction > 1.0f) deployedFraction = 1.0f;
  // TODO: implement servo write
  (void)deployedFraction;
}

/* models */
float accelModel(float t, float a, float custom_t_apog) {
  float dt = t - custom_t_apog;
  return a * pow5f_fast(dt) - g;
}

float getR2fromFit_accel(const AirbrakesAccelerationMeasurement *data,
                                size_t n,
                                float a,
                                float custom_t_apog)
{
  if (!data || n == 0) return 0.0f;

  float ss_res = 0.0f;
  float ss_tot = 0.0f;
  float sum_y  = 0.0f;

  for (size_t i = 0; i < n; i++) {
    float y  = data[i].accelerationMeasurement;
    float yh = accelModel(data[i].timeStamp, a, custom_t_apog);
    float r  = y - yh;
    ss_res += r*r;
    sum_y  += y;
  }

  float mean = sum_y / (float)n;
  for (size_t i = 0; i < n; i++) {
    float d = data[i].accelerationMeasurement - mean;
    ss_tot += d*d;
  }

  if (ss_tot == 0.0f) return 0.0f;
  return 1.0f - (ss_res / ss_tot);
}

int argmax(const float *arr, size_t n) {
  if (!arr || n == 0) return -1;
  float best = arr[0];
  int idx = 0;
  for (size_t i = 1; i < n; i++) {
    if (arr[i] > best) { best = arr[i]; idx = (int)i; }
  }
  return idx;
}

/* printing helper */
void printFloatArray3(const char *label, const float a[3]) {
  Serial.print(label);
  Serial.print("[");
  Serial.print(a[0], 4); Serial.print(", ");
  Serial.print(a[1], 4); Serial.print(", ");
  Serial.print(a[2], 4);
  Serial.println("]");
}

/* calculate the area needed for airbrakes */
float reqDeployedAreaAirbrakes(float t_0, float deltaX)
{
  float a  = coeffA;
  float b  = coeffB;
  float dt = (t_0 - t_apog);

  float a2 = a*a;
  float a3 = a2*a;
  float b2 = b*b;
  float b3 = b2*b;
  float g2 = g*g;
  float g3 = g2*g;

  float term =
      (a3)   * p10(dt) / 10.0f
    + (a2*b) * p9(dt)  / 3.0f
    + (3.0f*a*b2 - 3.0f*a2*g) * p8(dt) / 8.0f
    + (b3 - 6.0f*a*b*g)       * p7(dt) / 7.0f
    + (a*g2 - b2*g)           * p6(dt) / 2.0f
    + (3.0f*b*g2)             * p5(dt) / 5.0f
    - (g3)                    * p4(dt) / 4.0f;

  float xi = -term;

  float local_fudge = (deltaX > 40.0f) ? fudge_factor : fudge_factor_2;

  float denom = airbrakesCd * rho * xi;
  if (denom == 0.0f) return 0.0f;
  if (a_max == 0.0f) return 0.0f;

  float a_0 = local_fudge * 2.0f * mass * g * deltaX / denom;
  return maxf(0.0f, a_0 / a_max);
}

/* estimates */
float getVelocityEstimate(float t)
{
  float dt = t - t_apog;
  return coeffA * cubef_local(dt)
       + coeffB * sqf_local(dt)
       - g * dt;
}

float getAltitudeEstimate(float t, float alt0_local)
{
  float dt = t - t_apog;
  return coeffA * pow4f_local(dt) / 4.0f
       + coeffB * cubef_local(dt) / 3.0f
       - g * sqf_local(dt) / 2.0f
       + alt0_local;
}

float getAltitudeEstimate(float t)
{
  return getAltitudeEstimate(t, alt0);
}

/* start conditions */
bool shouldStartAirbrakesControlPrep() {
  return (getFlightTime() > EARLIEST_START_AIRBRAKES_PREP_TIME) &&
         (!apogeeReached) &&
         (currentRocketVel < START_AIRBRAKES_PREP_VEL);
}

bool shouldStartAirbrakesControlPreprocess() {
  return (getFlightTime() > START_AIRBRAKES_PREPROC_TIME) &&
         (!apogeeReached);
}

void handleAirbrakesState() {
  RocketStatus status;
  status.altitude      = 0.0f;
  status.vel_z         = currentRocketVel;
  status.accel_z       = currentRocketAccel;
  status.apogeeReached = apogeeReached;

  float currentTime = getFlightTime();

  apogeeReached      = status.apogeeReached;
  currentRocketVel   = status.vel_z;
  currentRocketAccel = status.accel_z;

  const int everyHowMany  = 1000 / AIRBRAKES_MEASUREMENT_FREQ_HZ;
  const int nMeasurements = AIRBRAKES_N_MEASUREMENTS;

  // DISABLED : awaiting start of prep (data collection)
  if (state == DISABLED) {
    state = shouldStartAirbrakesControlPrep() ? PREP : DISABLED;
    if (state == PREP) {
      datIndex = 0;
      counter = 0;
      Serial.println("[Airbrakes] Entering PREP");
    }
  }

  else if (state == PREP) {
    if ((counter % everyHowMany) == 0) {
      if (datIndex < nMeasurements) {
        accelData[datIndex] = AirbrakesAccelerationMeasurement_init(currentTime, currentRocketAccel);
        velData[datIndex]   = AirbrakesVelocityMeasurement_init(currentTime, currentRocketVel);
        datIndex++;
      }
    }

    counter++;

    if (datIndex >= nMeasurements) {
      state = PREPROCESS;
      counter = 0;
      Serial.println("[Airbrakes] Entering PREPROCESS (data collected)");
    } else {
      state = shouldStartAirbrakesControlPreprocess() ? PREPROCESS : PREP;
      if (state == PREPROCESS) {
        counter = 0;
        Serial.println("[Airbrakes] Entering PREPROCESS (time's up)");
      }
    }
  }

  // PREPROCESS: Predict Apogee, t_apog, and A0_req
  else if (state == PREPROCESS) {
    const float t_apog_trials[3] = {
      AIRBRAKES_SIMULATION_T_APOG - 1.0f,
      AIRBRAKES_SIMULATION_T_APOG,
      AIRBRAKES_SIMULATION_T_APOG + 1.0f
    };

    float resulting_R2_values[3] = {0.0f, 0.0f, 0.0f};

    for (size_t i = 0; i < 3; i++) {
      float sum_num = 0.0f;
      float sum_den = 0.0f;

      for (size_t j = 0; j < (size_t)nMeasurements; j++) {
        float dt = accelData[j].timeStamp - t_apog_trials[i];
        sum_den += pow10f_fast(dt);
        sum_num += (accelData[j].accelerationMeasurement + g) * pow5f_fast(dt);
      }

      float a_coeff = (sum_den != 0.0f) ? (sum_num / sum_den) : 0.0f;
      resulting_R2_values[i] = getR2fromFit_accel(accelData, (size_t)nMeasurements, a_coeff, t_apog_trials[i]);
    }

    printFloatArray3("[Airbrakes] t_apog trials: ", t_apog_trials);
    printFloatArray3("[Airbrakes] R2 values:    ", resulting_R2_values);

    int best_idx = argmax(resulting_R2_values, 3);
    float best_t = (best_idx >= 0) ? t_apog_trials[(size_t)best_idx] : AIRBRAKES_SIMULATION_T_APOG;
    t_apog = best_t + AIRBRAKES_T_APOG_FUDGEDIFF;

    Serial.print("[Airbrakes] Choosing t_apog = ");
    Serial.println(t_apog, 4);

    // Fit velocity coefficients
    float XT_X[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    float XT_y[2] = {0.0f, 0.0f};

    for (int i = 0; i < nMeasurements; i++) {
      float dt  = velData[i].timeStamp - t_apog;
      float dt2 = dt*dt;
      float dt3 = dt2*dt;
      float dt4 = dt2*dt2;
      float dt5 = dt4*dt;
      float dt6 = dt3*dt3;

      XT_X[0][0] += dt6;
      XT_X[0][1] += dt5;
      XT_X[1][0] += dt5;
      XT_X[1][1] += dt4;

      float vi = velData[i].velocityMeasurement;
      float yi = vi + g*dt;

      XT_y[0] += dt3 * yi;
      XT_y[1] += dt2 * yi;
    }

    float XT_X_inv[2][2];
    if (!inverse2x2Matrix(XT_X, XT_X_inv)) {
      Serial.println("[Airbrakes] XT_X singular -> INFEASIBLE");
      state = INFEASIBLE;
      return;
    }

    coeffA = XT_X_inv[0][0] * XT_y[0] + XT_X_inv[0][1] * XT_y[1];
    coeffB = XT_X_inv[1][0] * XT_y[0] + XT_X_inv[1][1] * XT_y[1];

    Serial.print("[Airbrakes] Velocity fit: a="); Serial.print(coeffA, 6);
    Serial.print(" b="); Serial.println(coeffB, 6);

    alt0 = status.altitude - getAltitudeEstimate(getFlightTime());
    predictedAlt = getAltitudeEstimate(t_apog);

    float desiredAlt = floorf(predictedAlt / (float)roundToHowMuch) * (float)roundToHowMuch;
    desiredDeltaX = predictedAlt - desiredAlt;

    A0_req = reqDeployedAreaAirbrakes(currentTime + AIRBRAKES_TIME_DELAY, desiredDeltaX);
    airbrakesCtrlStartTime = currentTime + AIRBRAKES_TIME_DELAY;

    if (A0_req > 1.0f) {
      Serial.print("[Airbrakes] Req A="); Serial.print(A0_req, 3);
      Serial.println(" > 1.0 (infeasible)");
      if (DEBUG_AIRBRAKES_ON) {
        A0_req = 1.0f;
        state = WAIT_FOR_START;
      } else {
        state = INFEASIBLE;
      }
    } else {
      Serial.println("[Airbrakes] Reaching Apogee is Feasible");
      Serial.print("A0_req="); Serial.println(A0_req, 3);
      Serial.print("[Airbrakes] Desired start time: "); Serial.println(airbrakesCtrlStartTime, 3);
      state = WAIT_FOR_START;
    }
  }
  // Wait for the designated start time.
  else if (state == WAIT_FOR_START) {
    if (getFlightTime() >= airbrakesCtrlStartTime) state = CONTROLLING_RAMP;
  }
  // Ramp up
  else if (state == CONTROLLING_RAMP) {
    float t = getFlightTime();
    if (t >= airbrakesCtrlStartTime) {
      float deployedFraction = 2.0f * A0_req * (t - airbrakesCtrlStartTime);
      setAirbrakesServo(deployedFraction);
    }
    if (t >= airbrakesCtrlStartTime + 0.5f) {
      state = CONTROLLING_PLATEAU;
      setAirbrakesServo(A0_req);
    }
  }
  // plateau
  else if (state == CONTROLLING_PLATEAU) {
    setAirbrakesServo(A0_req);
    if (status.vel_z <= 0.0f || status.apogeeReached) {
      state = DONE;
      setAirbrakesServo(0.0f);
    }
  }
  // DONE / INFEASIBLE: do nothing
}




/* ------------------ Arduino entry points ------------------ */
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Airbrakes controller starting...");
}

void loop() {
  // Replace with real telemetry
  if(millis()-lastTimeStamp >= 1000.0f/LOOP_FREQ) {

    // READ TELEMETRY

    handleAirbrakesState();
    lastTimeStamp = millis();
  }
  
}