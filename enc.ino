/*
 * http://www.picfun.com/motor05.html
 * https://www.esp32.com/viewtopic.php?t=1206
 */

const int PIN_PHASE_A = 7; // interrupt 0
const int PIN_PHASE_B = 3; // interrupt 1
const int PIN_MOTOR = 2;
const double coeff_step2rad = 0.01017684695040425409284950885416; // 0.01017684695040425409284950885416 // 2PI / (12*51.45)
const double target_speed = 31.4; // rad/sec
const double Kp = 0.001;
const double Ki = 0.1;
const double Kd = 0;
const int MAX_DU = 10;
const double pi = 3.1415926535897932384626;

long enc_abs, enc_abs_old;
double theta, theta_speed, theta_pre;

double dev, dev_pre, dev_prepre;
unsigned long time_now, time_pre;

int8_t oldA, oldB;
int8_t enc_ref[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

int counter;
long u;

void update_enc(void)
{
    int8_t newA = digitalRead(PIN_PHASE_A);
    int8_t newB = digitalRead(PIN_PHASE_B);

    enc_abs_old = enc_abs;

    int enc_inc = enc_ref[ ( oldA << 3 )
                        | ( oldB << 2 )
                        | ( newA << 1 )
                        | ( newB ) ];
    
    enc_abs += enc_inc;
    theta = coeff_step2rad * enc_abs;

    oldA = newA;
    oldB = newB;
}

void setup()
{
    attachInterrupt(digitalPinToInterrupt(PIN_PHASE_A), update_enc, CHANGE); //
    attachInterrupt(digitalPinToInterrupt(PIN_PHASE_B), update_enc, CHANGE);

    Serial.begin(9600);
}

void loop()
{
    
    time_pre = time_now;
    time_now = micros();
    unsigned long time_d = time_now - time_pre;

    theta_speed = ( theta - theta_pre ) / ( time_d * 0.000001);
    theta_pre = theta;

    dev_prepre = dev_pre;
    dev_pre = dev;
    dev = target_speed - theta_speed;
    int du = Kp * ( dev - dev_pre ) + Ki * dev * time_d + Kd * ((dev - dev_pre) - (dev_pre - dev_prepre))*time_d*time_d;
    if ( du > MAX_DU ) du = MAX_DU;
    else if ( du < -MAX_DU ) du = -MAX_DU;
    u += du;
    if ( u > 1023 ) u = 1023;
    else if ( u < 0 ) u = 0;

    // ledcWrite(1, u);
    analogWrite(PIN_MOTOR, u);
    
    
        
        Serial.print("target speed : ");
        Serial.print(target_speed);
        
        Serial.print(", speed : ");
        Serial.print(theta_speed);
        
        Serial.print(", theta : ");
        Serial.print(theta);
        Serial.print(", enc :");
        Serial.print(enc_abs);
        Serial.print(", u:");
        Serial.print(u);
        Serial.print("\n");

    delay(10);
}

