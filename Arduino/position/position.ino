#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

double xPos = 0, yPos = 0, headingVel = 0;
double S_gps_x, S_gps_y, S_gps_z = 0.0;// initial location
double Vx, Vy, Vz = 0.0; // initial velocity
double Vx_before, Vy_before, Vz_before;
double high_Vx, high_Vy, high_Vz;
double high_S_gps_x, high_S_gps_y, high_S_gps_z;
double S_gps_x_before, S_gps_y_before, S_gps_z_before;
double a[3], high_a[3], a_before[3], w[3], h[3], S[3], An[3];
double pi, theta, psi = 0.0;
double high_pi, high_theta, high_psi = 0.0;
double pi_before, theta_before, psi_before = 0.0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 100; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double t = ACCEL_VEL_TRANSITION;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!
  
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }


  delay(1000);
  uint8_t system, gyro, accel, mag = 0;
  while(system  != 3 || gyro != 3 || accel != 3){
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println("CALIBRATING...");
    Serial.print("Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    Serial.println("--");
    delay(100);
  }
}

void loop(void)
{
  //
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData, magnetometerData, angVelocityData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); 
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  //Raw data
  a[0] = linearAccelData.acceleration.x;
  a[1] = linearAccelData.acceleration.y;
  a[2] = linearAccelData.acceleration.z;
  // a[0] = acc.x();
  // a[1] = acc.y();
  // a[2] = acc.z();
  w[0] = angVelocityData.gyro.x;
  w[1] = angVelocityData.gyro.y;
  w[2] = angVelocityData.gyro.z;
  An[0] = euler.x();
  An[1] = euler.y();
  An[2] = euler.z();
  h[0] = magnetometerData.magnetic.x;
  h[1] = magnetometerData.magnetic.y;
  h[2] =  magnetometerData.magnetic.z;


  double acc_magnitude = sqrt(pow(a[0],2) + pow(a[1],2) + pow(a[2],2));
  // if(acc_magnitude-9.806 < 0.3){
  //     a[0] = 0;
  //     a[1] = 0;
  //     a[2] = 0;
  // }
  // //angular-drift compensation
  if (abs(An[0]) < 0.1) {
      An[0] = 0;
  }
  if (abs(An[1]) < 0.1) {
      An[1] = 0;
  }
  if (abs(An[2]) < 0.1) {
      An[2] = 0;
  }

  // pi_before = pi;
  // theta_before = theta;
  // psi_before = psi;

  // pi = An[0]*DEG_2_RAD;
  // theta = An[1]*DEG_2_RAD;
  // psi = An[2]*DEG_2_RAD;
  
  // high_pi = highpassfilter(pi_before, pi, high_pi);
  // high_theta = highpassfilter(theta_before, theta, high_theta);
  // high_psi = highpassfilter(psi_before, psi, high_psi);

  // //h-drift compensation
  // if (abs(h[0]) < 0.01) {
  //     h[0] = 0;
  // }
  // if (abs(h[1]) < 0.01) {
  //     h[1] = 0;
  // }
  // if (abs(h[2]) < 0.01) {
  //     h[2] = 0;
  // }

  
  // double acc_x_world = cos(high_theta) * cos(high_psi) * a[0] + (sin(high_pi) * sin(high_theta) * cos(high_psi) - cos(high_pi) * sin(high_psi)) * a[1] + (cos(high_pi) * sin(high_theta) * cos(high_psi) + sin(high_pi) * sin(high_psi)) * a[2];
  // double acc_y_world = cos(high_theta) * sin(high_psi) * a[0] + (sin(high_pi) * sin(high_theta) * sin(high_psi) + cos(high_pi) * cos(high_psi)) * a[1] + (cos(high_pi) * sin(high_theta) * sin(high_psi) - sin(high_pi) * cos(high_psi)) * a[2];
  // double acc_z_world = -sin(high_theta) * a[0] + sin(high_pi) * cos(high_theta) * a[1] + cos(high_pi) * cos(high_theta) * a[2]; //- acc_magnitude;
  
  // high_a[0] = highpassfilter(a_before[0], a[0], high_a[0]);
  // high_a[1] = highpassfilter(a_before[1], a[1], high_a[1]);
  // high_a[2] = highpassfilter(a_before[2], a[2], high_a[2]);


  double acc_x_world = a[0];
  double acc_y_world = a[1];
  double acc_z_world = a[2];

  // //V-drift compensation
  double acc_sqrt = sqrt(pow(acc_x_world, 2) + pow(acc_y_world, 2) + pow(acc_z_world, 2));
  if (acc_sqrt < 0.004) {
      acc_x_world = 0;
      acc_y_world = 0;
      acc_z_world = 0;
  }

  //예전 코드

  

  // Vx = Vx + acc_x_world * t;
  // Vy = Vy + acc_y_world * t;
  // Vz = Vz + acc_z_world * t;

  
  // S_gps_x = S_gps_x + Vx * t;
  // S_gps_y = S_gps_y + Vy * t;
  // S_gps_z = S_gps_z + Vz * t;

  // S[0] = S_gps_x;
  // S[1] = S_gps_y;
  // S[2] = S_gps_z;


  //high pass filter for V

  Vx_before = Vx;
  Vy_before = Vy;
  Vz_before = Vz;

  Vx = Vx + acc_x_world * t;
  Vy = Vy + acc_y_world * t;
  Vz = Vz + acc_z_world * t;

  //Highpassfilter(x1, x2, y1, cutoff)
  high_Vx = highpassfilter(Vx_before, Vx, high_Vx, 0.4);
  high_Vy = highpassfilter(Vy_before, Vy, high_Vy, 0.4);
  high_Vz = highpassfilter(Vz_before, Vz, high_Vz, 0.4);
  
  //high pass filter for S

  S_gps_x_before = S_gps_x;
  S_gps_y_before = S_gps_y;
  S_gps_z_before = S_gps_z;

  S_gps_x = S_gps_x + high_Vx * t;
  S_gps_y = S_gps_y + high_Vy * t;
  S_gps_z = S_gps_z + high_Vz * t;

  high_S_gps_x = highpassfilter(S_gps_x_before, S_gps_x, high_S_gps_x, 1);
  high_S_gps_y = highpassfilter(S_gps_y_before, S_gps_y, high_S_gps_y, 1);
  high_S_gps_z = highpassfilter(S_gps_z_before, S_gps_z, high_S_gps_z, 1);

  S[0] = high_S_gps_x;
  S[1] = high_S_gps_y;
  S[2] = high_S_gps_z;


  


  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    // Serial.print("Heading:\n| x: ");
    // Serial.println(An[0]);
    // Serial.print("| y: ");
    // Serial.println(An[1]);
    // Serial.print("| z: ");
    // Serial.println(An[2]);
    // Serial.print("Linear:\n| x: ");
    // Serial.println(acc_x_world);
    // Serial.print("| y: ");
    // Serial.println(acc_y_world);
    // Serial.print("| z: ");
    // Serial.println(acc_z_world);
    Serial.print("Position,");
    Serial.print(S[0]);
    Serial.print(",");
    Serial.print(S[1]);
    Serial.print(",");
    Serial.print(S[2]);
    Serial.print(",Orientation,");
    Serial.print(An[0]);
    Serial.print(",");
    Serial.print(An[1]);
    Serial.print(",");
    Serial.println(An[2]);
    // Serial.print("T: ");
    // Serial.println(t);
    // Serial.println(headingVel);
    // Serial.println("-------");
    // printEvent(&gravityData);
    printCount = 0;

  }
  else {
    printCount = printCount + 1;
  }



  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}

void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Linear_Acceleration:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orientation:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}


double lowpassfilter(double sensor, double result_before){
  //alpha = (1/(0.02*3.141592))/(1/(0.02*3.141592)+1/256)
  // int alpha = 1;
  // double result2 = alpha * ( result1 + sensor2 - sensor1);
  double fc = 1/10;
  double lambda = 2*3.141592*fc*t;
  double result = lambda/(1+lambda)*sensor+1/(1+lambda)*result_before;
  return result;
}

float highpassfilter(float sensor1, float sensor2, float result_before, float CUTOFF){
  //alpha = (1/(0.02*3.141592))/(1/(0.02*3.141592)+1/256)
  float RC = 1.0/(CUTOFF*2*3.141592);
  float alpha = RC/(RC+t);
  float result = alpha * ( result_before + sensor2 - sensor1);
  // double fc = 1/10;
  // double lambda = 2*3.141592*fc*t;
  // double result = lambda/(1+lambda)*sensor+1/(1+lambda)*result_before;
  return result;
}


