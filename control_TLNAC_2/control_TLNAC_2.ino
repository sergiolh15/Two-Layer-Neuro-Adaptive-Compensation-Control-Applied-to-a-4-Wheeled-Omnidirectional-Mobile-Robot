 /*******************************************************
 * Project: Two-Layer Neuro-Adaptive Compensation (TLNAC)
 * Robot: Nexus 4-WOMR (4-Wheeled Omnidirectional Mobile Robot)
 * Controller: TLNAC real-time implementation
 *
 * Author: Sergio Lopez
 * Affiliation: Division of Graduate Studies and Research,
 *              Tecnológico Nacional de México / Instituto Tecnológico de La Laguna
 * Email: slopezh@lalaguna.tecnm.mx
 *
 * Co-Author: Miguel A. Llama
 * Affiliation: Division of Graduate Studies and Research,
 *              Tecnológico Nacional de México / Instituto Tecnológico de La Laguna
 * Email: mllama@lalaguna.tecnm.mx
 *
 * Description:
 * This Arduino sketch implements the TLNAC control scheme for real-time
 * wheel speed regulation based on inverse kinematics of the Nexus 4-WOMR.
 *
 * Hardware: ATmega328p microcontroller on the Nexus 4-WOMR
 * Libraries: List any required Arduino libraries here (Servo, Wire, etc.)
 *
 * License: MIT License
 * Date: 2025
 * See https://github.com/sergiolh15/Two-Layer-Neuro-Adaptive-Compensation-Control-Applied-to-a-4-Wheeled-Omnidirectional-Mobile-Robot and the README.md for more information.
 *******************************************************/
#include <EnableInterrupt.h>
//Llanta 1
#define PWM1 10
#define DIR1 7 //HIGH: POSITIVO, LOW: NEGATIVO
#define APIN1 5
#define BPIN1 6
//Llanta 2
#define PWM2 3 
#define DIR2 2 //LOW: POSITIVO, HIGH: NEGATIVO
#define APIN2 4
#define BPIN2 18
//Llanta3
#define PWM3 11
#define DIR3 12//LOW: POSITIVO, HIGH: NEGATIVO
#define APIN3 14
#define BPIN3 15
//Llanta4
#define PWM4 9 
#define DIR4 8 //HIGH: POSITIVO, LOW: NEGATIVO
#define APIN4 16
#define BPIN4 17


//Declaracion
volatile int16_t intCount1 = 0; // The count will go back to 0 after hitting 65535.
volatile int16_t intCount2 = 0;
volatile int16_t intCount3 = 0;
volatile int16_t intCount4 = 0;
float vel[] = {0, 0, 0, 0};
float veld[] = {9.5, 9.5, 9.5, 9.5};
float velf[] = {0, 0, 0, 0};
float error[4];
float qpd[3];
float inte[4] = {0};
float r[4] = {0}, s[4], x[3][4]={0};
float phi[2][4], vtx[2][4];
float f[4] = {0};
float w[2][4] = {0};//{{0.01,0.01,0.01,0.01},{0.01,0.01,0.01,0.01}}; 
float v[2][3]={{0.0975,0.5469,0.9649},{0.2785,0.9575,0.1576}};
//float v[4][2][3]={{{0.0975,0.5469,0.9649},{0.2785,0.9575,0.1576}},{{0.0975,0.5469,0.9649},{0.2785,0.9575,0.1576}},{{0.0975,0.5469,0.9649},{0.2785,0.9575,0.1576}},{{0.0975,0.5469,0.9649},{0.2785,0.9575,0.1576}}};
int16_t tao[4] = {0};
//float kp = 5, kd = 0.25, G = 0.7;//kp0.5 kd0.025 g0.01 piso filtro kp5 kd0.25 g0.1
float kp = 25, ki = 150, G=0, E=0.001;//G1 = 0.05, G2=0.05;//G=0.5;kp0.5 kd0.025 g0.05 piso filtro kp5 kd0.25 g0.1
float l1=0.1524, l2=0.1505, th=0, ra=0.05;
unsigned long samp = 0, t; //tiempo de muestreo
uint8_t i, j, k;
int ledState = LOW;
//long t;
//Funciones de interrupcion
void intFunction1() {
  int b = digitalRead(BPIN1);
  if(b > 0)      intCount1++;
  else      intCount1--;
}
void intFunction2() {
  int b = digitalRead(BPIN2);
  if(b > 0)      intCount2--;
  else      intCount2++;
}
void intFunction3() {
  int b = digitalRead(BPIN3);
  if(b > 0)      intCount3--;
  else      intCount3++;
}
void intFunction4() {
  int b = digitalRead(BPIN4);
  if(b > 0)      intCount4++;
  else      intCount4--;
}


void setup() {
  Serial.begin(57600);//Velocidad de comunicacion 
  TCCR1B = TCCR1B & 0xf8 | 0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz
  pinMode(APIN1, INPUT_PULLUP);//Activacion de reistencias pull-up
  pinMode(APIN2, INPUT_PULLUP);
  pinMode(APIN3, INPUT_PULLUP);
  pinMode(APIN4, INPUT_PULLUP);
  pinMode(BPIN1, INPUT_PULLUP);//Activacion de reistencias pull-up
  pinMode(BPIN2, INPUT_PULLUP);
  pinMode(BPIN3, INPUT_PULLUP);
  pinMode(BPIN4, INPUT_PULLUP);
  enableInterrupt(APIN1, intFunction1, RISING);//Asignacion de interrupciones //CHANGE
  enableInterrupt(APIN2, intFunction2, RISING);
  enableInterrupt(APIN3, intFunction3, RISING);
  enableInterrupt(APIN4, intFunction4, RISING);
  /*while(!(Serial.available()>=1 )){}//&& Serial.read() == 'S'
  G=float(Serial.read())/100;//kp=Serial.read();
  delay(15);
  samp=millis();*/
}

void loop() {
//Inicio del loop que se realiza cada tiempo de muestreo
  if (millis() >= samp) { 
    t=millis();//para calcular tiempos
    noInterrupts();
//Calculo de velocidades, formula => (Cuentas*2*pi)/(ranuras*relacion de engrane*tiempo de muestreo)=>Radianes/segundo
    vel[0] = (float)intCount1 / 12.2231; 
    vel[1] = (float)intCount2 / 12.2231;
    vel[2] = (float)intCount3 / 12.2231;
    vel[3] = (float)intCount4 / 12.2231;
    intCount1 = 0;//resetear las cuentas para el siguiente calculo
    intCount2 = 0;
    intCount3 = 0;
    intCount4 = 0;
    interrupts();
//Filtro de primer orden para la medicion de velocidad
    /*velf[0] += 0.1*20*(vel[0] - velf[0]);
    velf[1] += 0.1*20*(vel[1] - velf[1]);
    velf[2] += 0.1*20*(vel[2] - velf[2]);
    velf[3] += 0.1*20*(vel[3] - velf[3]);*/
//Calculo de referencias
    //if ((millis()/1000<15)||millis()/1000>30)veld[0]=9.5;
    //else veld[0]=7.5;
    //veld[0]=6.5*(1-exp(-0.2*millis()/1000))+4*abs(sin(2*PI*millis()/60000));
    //veld[0]=5*2*cos(2*PI*millis()/15000+2*PI*1.85/15);
    //veld[1]=5*2*cos(2*PI*millis()/15000-2*PI*1.85/15);
    qpd[0] =  2*PI*cos(2*PI*millis()/15000)/15;
    qpd[1] = -2*PI*sin(2*PI*millis()/15000)/15;
    qpd[2] =  -2*PI/15;
    th     =  -float(millis())*2*PI/15000;
    //Serial.println(String(millis()/1000));
    //Cinematica => vphipd=r*E*(R')*qpd;
    veld[0] = (qpd[0]*(cos(th) - sin(th)) + qpd[1]*(cos(th) + sin(th)) + qpd[2]*(l1 + l2))/ra;
    veld[1] = (qpd[0]*(cos(th) + sin(th)) - qpd[1]*(cos(th) - sin(th)) - qpd[2]*(l1 + l2))/ra;
    veld[2] = (qpd[0]*(cos(th) - sin(th)) + qpd[1]*(cos(th) + sin(th)) - qpd[2]*(l1 + l2))/ra;
    veld[3] = (qpd[0]*(cos(th) + sin(th)) - qpd[1]*(cos(th) - sin(th)) + qpd[2]*(l1 + l2))/ra;
//Calculo de errores de velocidad 
    error[0] = veld[0] - vel[0];
    error[1] = veld[1] - vel[1];
    error[2] = veld[2] - vel[2];
    error[3] = veld[3] - vel[3];
//Calculo de la derivada del error
    /*dere[0] = (error[0] - error0[0]) / 0.1;
    dere[1] = (error[1] - error0[1]) / 0.1;
    dere[2] = (error[2] - error0[2]) / 0.1;
    dere[3] = (error[3] - error0[3]) / 0.1;*/
//Calculo de la integral del error
    inte[0] += 0.1*error[0];
    inte[1] += 0.1*error[1];
    inte[2] += 0.1*error[2];
    inte[3] += 0.1*error[3];
//Calculo del filtrado del error
    r[0] = error[0] + ki * inte[0] / kp; //filtrado del error
    r[1] = error[1] + ki * inte[1] / kp;
    r[2] = error[2] + ki * inte[2] / kp;
    r[3] = error[3] + ki * inte[3] / kp;
//Calculo del vector de entrada
    for (i = 0; i < 4; i++) {
      x[i][0]=inte[i];
      x[i][1]=error[i];
      x[i][2]=1;
    }
//Calculo del vector Phi(V^T x) (funcion de activacion) para cada llanta
    for (i = 0; i < 2; i++) 
      for (j = 0; j < 4; j++) 
        vtx[i][j]=0;//Inicializando

    for (k = 0; k < 4; k++)
      for (i = 0; i < 2; i++) 
        for (j = 0; j < 3; j++)
          vtx[i][k]+=v[i][j]*x[j][k];//Calculando V^T x 

    for (i = 0; i < 2; i++) 
      for (j = 0; j < 4; j++) 
        phi[i][j]=tanh(vtx[i][j]);//Calculando tanh(V^T x)
//Calculo de los pesos V de cada red neuronal
    /*for (k = 0; k < 4; k++)
      for (i = 0; i < 2; i++) 
        for (j = 0; j < 3; j++){ 
          v[k][i][j]+=0.1*G*x[j][k]*r[k]*w[i][k]*(1-phi[i][k]*phi[i][k]);
        }*/
//Calculo de los pesos W de cada red neuronal
    for (i = 0; i < 2; i++) { 
      w[i][0] += 0.1 * G * (phi[i][0] - (1-phi[i][0]*phi[i][0])*vtx[i][0])* r[0];
      w[i][1] += 0.1 * G * (phi[i][1] - (1-phi[i][1]*phi[i][1])*vtx[i][1])* r[1];
      w[i][2] += 0.1 * G * (phi[i][2] - (1-phi[i][2]*phi[i][2])*vtx[i][2])* r[2];
      w[i][3] += 0.1 * G * (phi[i][3] - (1-phi[i][3]*phi[i][3])*vtx[i][3])* r[3];

      /*if (w[i][0]>=65)w[i][0]=65;
      else if (w[i][0]<=-65)w[i][0]=-65;
      if (w[i][1]>=65)w[i][1]=65;
      else if (w[i][1]<=-65)w[i][1]=-65;
      if (w[i][2]>=65)w[i][2]=65;
      else if (w[i][2]<=-65)w[i][2]=-65;
      if (w[i][3]>=65)w[i][3]=65;
      else if (w[i][3]<=-65)w[i][3]=-65;*/
    }
//Calculo de la funcion F y el signo
    for (i = 0; i < 4; i++)      f[i] = 0;
    for (i = 0; i < 4; i++) {
      f[0] += phi[i][0] * w[i][0];
      f[1] += phi[i][1] * w[i][1];
      f[2] += phi[i][2] * w[i][2];
      f[3] += phi[i][3] * w[i][3];
      s[i] = (r[i] > 0) - (r[i] < 0);//Función signo
    }
//Calculo de la ley de control (kp*r  =kp*(error[0]+kd*inte[0]/kp))
    tao[0] = kp * r[0] + f[0] +E*s[0];
    tao[1] = kp * r[1] + f[1] +E*s[1];
    tao[2] = kp * r[2] + f[2] +E*s[2];
    tao[3] = kp * r[3] + f[3] +E*s[3];
    //tao[0] = kp * r[0];
    //tao[1] = kp * r[1];
    //tao[2] = kp * r[2];
    //tao[3] = kp * r[3];    
//Saturaci[on de la salida (PWM)
    if (tao[0] > 254)      tao[0] = 254;
    else if (tao[0] < 0)      tao[0] = 0;
    if (tao[1] > 254)      tao[1] = 254;
    else if (tao[1] < 0)      tao[1] = 0;
    if (tao[2] > 254)      tao[2] = 254;
    else if (tao[2] < 0)      tao[2] = 0;
    if (tao[3] > 254)      tao[3] = 254;
    else if (tao[3] < 0)      tao[3] = 0;
//Asignacion de valores anteriores
    /*error0[0] = error[0];
    error0[1] = error[1];
    error0[2] = error[2];
    error0[3] = error[3];*/ 
//Sentido de giro y aplicacion de la salida
    if (tao[0] >= 0)    digitalWrite(DIR1, HIGH);
    else      digitalWrite(DIR1, LOW);
  
    if (tao[1] >= 0)    digitalWrite(DIR2, LOW);
    else      digitalWrite(DIR2, HIGH); 
        
    if (tao[2] >= 0)    digitalWrite(DIR3, LOW);
    else      digitalWrite(DIR3, HIGH);
    
    if (tao[3] >= 0)    digitalWrite(DIR4, HIGH);
    else      digitalWrite(DIR4, LOW);
    
    analogWrite(PWM1, abs(tao[0]));
    analogWrite(PWM2, abs(tao[1]));
    analogWrite(PWM3, abs(tao[2]));
    analogWrite(PWM4, abs(tao[3]));
// if the LED is off turn it on and vice-versa:
    ledState ^= (1 << 0);   
    // set the LED with the ledState of the variable:
    digitalWrite(13, ledState);
    
//Impresion de resultados
    //Serial.println(String(qpd[0])+ ", " + String(qpd[1])+ ", " + String(qpd[2]));
    //Serial.println(String(vel[0]) + ", " + String(vel[1]) + ", " + String(vel[2]) + ", " + String(vel[3]));
    //Serial.println(String(vel[0]) + ", " + String(vel[1]) + ", " + String(vel[2]) + ", " + String(vel[3]) + ", " + String(veld[0])+ ", " + String(veld[1]) + ", " + String(veld[2])+ ", " + String(veld[3]));
    //Serial.println(String(veld[0])  + ", " + String(veld[1]) + ", " + String(veld[2])  + ", " + String(veld[3])+ ", " + String(-11)  + ", " + String(11));
    //Serial.println(String(micros()));
    //Serial.println(String(micros()-t));
    Serial.print(float(t)/1000);Serial.print(", ");
    Serial.print(String(vel[0]) + ", " + String(vel[1]) + ", " + String(vel[2]) + ", " + String(vel[3]) + ", ");
    Serial.println(String(tao[0]) + ", " + String(tao[1]) + ", " + String(tao[2]) + ", " + String(tao[3])+ ", " + String(G));
    samp += 100;
  }
}
