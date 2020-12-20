// --------------------- НАСТРОЙКИ ----------------------
// MATRIX
#define PIN 6 // Din ribbon pin
#define BRIGHTNESS 150 // brightness (0 - 255)
#define MATR_X 16 // number of LEDs per x
#define MATR_Y 16 // number of LEDs by y
#define PIXELSIZE 5 // pixel size mm

// PHYSICS
#define G_CONST 9.81 // free fall acceleration
#define FRICTION 1 // friction
#define MIN_STEP 10 // minimum step of integration (milliseconds)
// with a strong decrease in the step, everything goes n * z here, which is very strange for Euler ...

// EFFECTS
#define PIXEL_AMOUNT 50 // number of "live" pixels
#define GLOW 0 // glow
#define ALL_BLUE 0 // all blue

// offsets for the accelerometer
//int offsets[6] = { -3214, -222, 1324, -2, -67, -12}; //Ori
//int offsets[6] = { -575, 1920, 488, 47, -24, 12}; //ADO Low 0x68  address pin low (GND)
int offsets[6] = { -582, 1927, 484, 42, -22, 11}; //ADO High 0x69  address pin high (VCC)

/*
  LOOK HOW THE BALLOONS ARE BEING ON OTHER PLANETS !!!
   Earth 9.81 m / s2
   Sun 273.1 m / s2
   Moon 1.62 m / s2
   Mercury 3.68 m / s2
   Venus 8.88 m / s2
   Mars 3.86 m / s2
   Jupiter 23.95 m / s2
   Saturn 10.44 m / s2
   Uranium 8.86 m / s2
   Neptune 11.09 m / s2
*/
// --------------------- SETTINGS ----------------------

// colors
/*
#define BLUE 0x000088
#define RED 0x880000
#define GREEN 0x00ff00
#define CYAN 0x008888
#define MAGENTA 0xaa0088
#define YELLOW 0x888800
#define WHITE 0x505050
*/
#define RED 0xFF0000
#define GREEN 0x008000
#define BLUE 0x0000FF
#define MAGENTA 0xFF00FF
#define YELLOW 0xFFFF00
#define CYAN 0x00FFFF
#define BLACK 0x000000
#define WHITE 0xFFFFFF
#define GRAY 0x808080
#define PURPLE 0x800080

#define GLOW_FADE1 0x000004 // near glow
#define GLOW_FADE2 0x000002 // distant glow

uint32_t colors[] = {
  RED,
  GREEN,
  BLUE,
  MAGENTA,
  YELLOW,
  CYAN,
  BLACK,
  WHITE,
  GRAY,
  PURPLE,
};

uint8_t hue = 0;

// ---------- LIBRARIES -----------
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MATR_X * MATR_Y, PIN, NEO_GRB + NEO_KHZ800);
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;
// ---------- LIBRARIES -----------

// ---------- OTHER -----------




// --------------------- FOR DEVELOPERS ----------------------
#define MATR_X_M MATR_X * PIXELSIZE // matrix size in millimeters x
#define MATR_Y_M MATR_Y * PIXELSIZE // matrix size in millimeters y

int x_vel [PIXEL_AMOUNT]; // MILLIMETERS PER SECOND
int y_vel [PIXEL_AMOUNT]; // MILLIMETERS PER SECOND
int x_dist [PIXEL_AMOUNT]; // MILLIMETERS
int y_dist [PIXEL_AMOUNT]; // MILLIMETERS
byte friction[PIXEL_AMOUNT];
byte bounce[PIXEL_AMOUNT];
byte color[PIXEL_AMOUNT];

float mpuPitch;
float mpuRoll;
int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long integrTimer, loopTimer;
float stepTime;
// --------------------- FOR DEVELOPERS ----------------------

void setup() {
  Serial.begin(9600);
  mpuSetup();
  strip.begin();
  strip.setBrightness(BRIGHTNESS);

  // initial conditions. Center of the matrix, speed zero
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {
    x_vel[i] = 0;
    y_vel[i] = 0;
    x_dist[i] = (MATR_X_M / 2);
    y_dist[i] = (MATR_Y_M / 2);
  }

  randomSeed (analogRead (0)); // seed for pseudo-random number generator

   for (byte i = 0; i <PIXEL_AMOUNT; i ++) {
     // get random values
     friction [i] = random (0, 30); // FRICTION. Further divisible by 100
     bounce [i] = random (60, 95); // Bounce. Further divisible by 100

     // tricky here so that the entire palette of colors goes to all pixels
    color[i] = map(i, 0, PIXEL_AMOUNT, 0, 7);
    if (ALL_BLUE) color[i] = 0;
  }
}

void loop() {
  if (millis() - loopTimer > MIN_STEP) {
    loopTimer = millis();
    integrate();
    strip.clear();
    for (byte i = 0; i < PIXEL_AMOUNT; i++) {
      byte nowDistX = floor (x_dist [i] / PIXELSIZE); // convert millimeters to pixels
      byte nowDistY = floor (y_dist [i] / PIXELSIZE); // convert millimeters to pixels

      if (GLOW) {
        glowDraw (nowDistX - 1, nowDistY, GLOW_FADE1); // draw a point
        glowDraw (nowDistX + 1, nowDistY, GLOW_FADE1); // draw a point
        glowDraw (nowDistX, nowDistY - 1, GLOW_FADE1); // draw a point
        glowDraw (nowDistX, nowDistY + 1, GLOW_FADE1); // draw a point

        glowDraw (nowDistX - 1, nowDistY + 1, GLOW_FADE2); // draw a point
        glowDraw (nowDistX - 1, nowDistY - 1, GLOW_FADE2); // draw a point
        glowDraw (nowDistX + 1, nowDistY - 1, GLOW_FADE2); // draw a point
        glowDraw (nowDistX + 1, nowDistY + 1, GLOW_FADE2); // draw a point

        glowDraw (nowDistX - 2, nowDistY, GLOW_FADE2); // draw a point
        glowDraw (nowDistX + 2, nowDistY, GLOW_FADE2); // draw a point
        glowDraw (nowDistX, nowDistY - 2, GLOW_FADE2); // draw a point
        glowDraw (nowDistX, nowDistY + 2, GLOW_FADE2); // draw a point
      }

      pixelDraw (nowDistX, nowDistY, color [i]); // draw a point

    }
    strip.show();
  }
}

//int pass = 0;
void pixelDraw (byte x, byte y, byte colorNum) {
  static int i = 0;
  i++;
  if (y% 2 == 0) // if even string
    //strip.setPixelColor(y * MATR_X + x, colors[colorNum]); 
    strip.setPixelColor(y * MATR_X + x, strip.gamma32(strip.ColorHSV((x + y) * 256 * 16 + i * 32 * 1))); 
  else // if odd
    //strip.setPixelColor(y * MATR_X + MATR_X - x - 1, colors[colorNum]);
    strip.setPixelColor(y * MATR_X + MATR_X - x - 1, strip.gamma32(strip.ColorHSV((x + y) * 256 * 16 + i * 32 * 1))); 
}
void glowDraw (byte x, byte y, byte color) {
  if (y% 2 == 0) // if even string
    strip.setPixelColor (y * MATR_X + x, color); // fill in direct order
  else // if odd
    strip.setPixelColor (y * MATR_X + MATR_X - x - 1, color); // fill in reverse order
}

void integrate() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                     // get accelerations
  mpuPitch = (float)ax / 16384;                                     // 16384 is the g value from the accelerometer
  mpuRoll = (float)ay / 16384;

  stepTime = (float)((long)millis() - integrTimer) / 1000;            // calculation of the integration step time
  integrTimer = millis();
  for (byte i = 0; i < PIXEL_AMOUNT; i++) {                           // for each pixel
    int thisAccel_x, thisAccel_y;
    int grav, frict;

    ///////////////////// AXIS X /////////////////////
    grav = (float)G_CONST * mpuPitch * 1000;    // сила тяжести
    if (FRICTION) {
      frict = (float)G_CONST * (1 - mpuPitch) * friction[i] * 10; // friction force
      if (x_vel[i] > 0) frict = -frict;   // the sign of the friction force depends on the direction of the velocity vector
      if (x_vel[i] == 0 && abs(grav) < frict) thisAccel_x = 0;  // static friction
      else thisAccel_x = (grav + frict);                        // acceleration
    } else thisAccel_x = grav;

    /////////////////////// AXIS Y /////////////////////
    grav = (float)G_CONST * mpuRoll * 1000;
    if (FRICTION) {
      frict = (float)G_CONST * (1 - mpuRoll) * friction[i] * 10;
      if (y_vel[i] > 0) frict = -frict;
      if (y_vel[i] == 0 && abs(grav) < frict) thisAccel_y = 0;
      else thisAccel_y = (grav + frict);
    } else thisAccel_y = grav;

    ///////////////////// WE INTEGRATE ///////////////////
     // speed at this step V = V0 + ax * dt
    x_vel[i] += (float)thisAccel_x * stepTime;
    y_vel[i] += (float)thisAccel_y * stepTime;

    // координата на данном шаге X = X0 + Vx*dt
    x_dist[i] += (float)x_vel[i] * stepTime;
    y_dist[i] += (float)y_vel[i] * stepTime;

    /////////////////// BEHAVIOR AT THE WALLS /////////////////
     // consider 4 walls of the matrix
    if (x_dist[i] < 0) {     // if you punched the edge of the matrix
      x_dist[i] = 0;         // return to the edge
      x_vel[i] = -x_vel[i] * (float)bounce[i] / 100;    // take the speed with the opposite sign and * for the rebound coefficient
    }
    if (x_dist[i] > MATR_X_M - PIXELSIZE) {
      x_dist[i] = MATR_X_M - PIXELSIZE;
      x_vel[i] = -x_vel[i] * (float)bounce[i] / 100;
    }

    if (y_dist[i] < 0) {
      y_dist[i] = 0;
      y_vel[i] = -y_vel[i] * (float)bounce[i] / 100;
    }
    if (y_dist[i] > MATR_Y_M - PIXELSIZE) {
      y_dist[i] = MATR_Y_M - PIXELSIZE;
      y_vel[i] = -y_vel[i] * (float)bounce[i] / 100;
    }
  }
}

void mpuSetup() {
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // set offsets
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[5]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[3]);
}
