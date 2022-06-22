#include <FirebaseESP32.h>
#include <WiFi.h>
#include <Wire.h>
#include "OakOLED.h"
#include "MAX30105.h"

TaskHandle_t Lectura;
#define pdTICKS_TO_MS

OakOLED oled;
FirebaseData firebaseData;
MAX30105 particleSensor;

#define FIREBASE_HOST "xxxxxx" 
#define FIREBASE_AUTH "xxxxxx"

#define WIFI_SSID "xxxxxx" //--------------------------------------------- Ingresar nombre de la red
#define WIFI_PASSWORD "xxxxx" //------------------------------------- Ingresar contraseña de la red

String usuario = "xxxxx"; //--------------------------------------- Ingresar usuario

String pathBPM = "/UsersList/"+usuario+"/MEDICIONES/Pulso";
String pathSpO2 = "/UsersList/"+usuario+"/MEDICIONES/Oximetria";
String pathConexion = "/UsersList/"+usuario+"/MEDICIONES/CONEXION";


//--------------------------------------------------------------------------------------- Variables Calculo SpO2 y Bpm
uint32_t irBuffer[50]; //infrared LED sensor data
uint32_t redBuffer[50];  //red LED sensor data
float ContRms = 0;
int FlagInit = 0;
int flag = 0; 
float HeartRate=0;
float SpO2 = 0;
float HeartRatePrev=0;
float ProgramInit=0;
float Signal=0;
float Alpha=0.6;
float s=0;
long w=0;
long wprev=0;
long y=0;
float rms=0;
float R=7.185;
int PulsesCount=0;
long IRw=0;
long IRwPrev=0;
long IRy=0;
float IRSignal=0;
float IRS=0;
float IRMS=0;
int BpmFire=0;
int SpO2Fire=0;
unsigned long iniciotiempoContador=0;

//--------------------------------------------------------------------------------------------------- ESTADOS
int EstadoWifi = 0;
#define ESTADO_WIFI_CONECTADO  1
#define ESTADO_WIFI_DESCONECTADO 2

int EstadoDedo = 0;
#define ESTADO_SIN_DEDO  1
#define ESTADO_CON_DEDO 2
int Std_Dedo_Cal = 0;

int StdMed1 = 0;
int StdMed2 = 0;
int StdMed3 = 0;
int StdMed4 = 0;
int StdMed5 = 0;

int F1 = 0;
int F2 = 0;
int F3 = 0;
int F4 = 0;
int F5 = 0;
int F1CW = 0;
int F2CW = 0;
int F3CW = 0;
int F4CW = 0;
int F5CW = 0;

//-------------------------------------------------------------------- tiempos de estado de Impresion pantalla
const long Interval_Oled_SW_MED_Std1 = 100;
unsigned long PreviousMillisMed1=0;
const long Interval_Oled_SW_MED_Std2 = 13000;
unsigned long PreviousMillisMed2=0;
const long Interval_Oled_SW_MED_Std3 = 25000;
unsigned long PreviousMillisMed3=0;
const long Interval_Oled_SW_MED_Std4 = 37000;
unsigned long PreviousMillisMed4=0;
const long Interval_Oled_SW_MED_Std5 = 40000;
unsigned long PreviousMillisMed5=0;

const long Interval_Control_Calculo1 = 100;        //---------------------------Controlar el tiempo de cálculo
unsigned long PreviousMillisCalculo1=0;
const long Interval_Control_Calculo2 = 39000;
unsigned long PreviousMillisCalculo2=0;
int FC1 = 0;
int FC2 = 0;
int StdC1 = 0;
int StdC2 = 0;

const long Interval_Control_Fire1 = 100;        //-------------------------Controlar el tiempo de envío Firebase
unsigned long PreviousMillisFire1 = 0;
const long Interval_Control_Fire2 = 1100;
unsigned long PreviousMillisFire2 = 0;
const long Interval_Control_Fire3 = 39000;
unsigned long PreviousMillisFire3 = 0;
const long Interval_Control_Fire4 = 40000;
unsigned long PreviousMillisFire4 = 0;
int FF1 = 0;
int FF2 = 0;
int StdF1 = 0;
int StdF2 = 0;
int StdF3 = 0;
int StdF4 = 0;

const long Interval_Ctrl_Cnx_Fire1 = 6000;        //------------------Controlar envío señal de conexion a Firebase
unsigned long PreviousMillisCnxFire1 = 0;

void setup() {
  
  xTaskCreatePinnedToCore(
    loop_lectura,
    "loop_lectura",
    10000,
    NULL,
    1,
    &Lectura,
    0);  
    
  Serial.begin(115200); 
  oled.begin();
  delay(100);

  oled.clearDisplay();
  oled.setTextSize(3);
  oled.setTextColor(1);
  oled.setCursor(37, 11);
  oled.println("UTN");
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(31, 39);
  oled.println("FACULTAD DE");
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(40, 49);
  oled.println("POSGRADO");
  oled.display();
  delay(5000);
  
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(1);
  oled.setCursor(4, 20);
  oled.println("SIGNAL-MED");
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(19, 38);
  oled.println("MAS CERCA DE TI");
  oled.display();
  delay(3000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //WiFi.begin(ssid, pass);
  //Serial.print("Conectando a ....");

  while (WiFi.status() != WL_CONNECTED)
  {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(1);
      oled.setCursor(0, 28);
      oled.println("Buscando Red WiFi...");
      oled.display();
      Serial.print("Buscando Red WiFi...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      delay(5000); 
  }

  Serial.println();
  Serial.print("Conectado con la IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("Inicializando Signal-Med");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(firebaseData, 1000 * 60); 
  Firebase.setwriteSizeLimit(firebaseData, "tiny"); //Tamaño y  tiempo de espera de escritura, tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setInt(firebaseData, pathConexion, 1);
  Firebase.setInt(firebaseData, pathBPM,0);
  Firebase.setInt(firebaseData, pathSpO2,0);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(15, 24);
  oled.println("Red identificada");
  oled.display();
  delay(1500);
  
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(34,0);
  oled.println("SIGNAL-MED");
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(110,0);
  oled.println(".il");
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(38,29);
  oled.println("Bienvenido");
  oled.drawLine(0, 10, 128, 10, 1);
  oled.display();
  delay(2000);
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)){ //Use default I2C port, 400kHz speed
    Serial.println("Error sensor");
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(34,0);
    oled.println("SIGNAL-MED");
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(110,0);
    oled.println(".il");
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(24,29);
    oled.println("Error sensor");
    oled.drawLine(0, 10, 128, 10, 1);
    oled.display();
    delay(4000);   
    //while (1); 
  }
  else {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(34,0);
    oled.println("SIGNAL-MED");
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(110,0);
    oled.println(".il");
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(33,29);
    oled.println("Sensor OK");
    oled.drawLine(0, 10, 128, 10, 1);
    oled.display();
    delay(2000);   

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(34,0);
    oled.println("SIGNAL-MED");
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(110,0);
    oled.println(".il");
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(19,29);
    oled.println("Coloque su dedo");
    oled.drawLine(0, 10, 128, 10, 1);
    oled.display();
    delay(2000);
  }
  particleSensor.setup(55, 4, 2, 200, 411, 4096);
  ProgramInit=millis();
  BpmFire = 0;
  SpO2Fire = 0;  
  redBuffer[1]=0;
  irBuffer[1]=0;   
}

void loop_lectura(void *pvParameters){  //======================================================================== FUNCION 1.CALCULAR

    while(1){
                           
      redBuffer[1]=particleSensor.getFIFORed();
      irBuffer[1]=particleSensor.getFIFOIR();     
    
      wprev=w;
      w=redBuffer[1]+(wprev*0.8); 
      y=w-wprev;
      
      Signal=float(y);
      s=(Alpha*Signal)+((1-Alpha)*s);    
                                
      IRwPrev=IRw;
      IRw=irBuffer[1]+(IRwPrev*0.8); 
      IRy=IRw-IRwPrev;
      
      IRSignal=float(IRy);
      IRS=(Alpha*IRSignal)+((1-Alpha)*IRS); 

      if((s>0)&&(flag==0)){
        PulsesCount+=1;
        flag=1;
      }
      if((s<0)&&(flag==1)){
        PulsesCount+=1;
        flag=0;
      }   
      if(PulsesCount>40){
        PulsesCount=0; 
      }   
     
    vTaskDelay( pdMS_TO_TICKS(100) );
  }
}

void Calcular(){ //================================================================================== FUNCION 1.CALCULAR

  ControlCalculo();
  
  while(Std_Dedo_Cal == 1){
    
    unsigned long currentMillisContador = millis();
    if(particleSensor.getRed()<70000){
      BpmFire = 0;
      SpO2Fire = 0;       
    }
    else{
             
      ContRms+=1;                                         
                        
      if((currentMillisContador-iniciotiempoContador)>=5000){                                  

        if(FlagInit==0){
          HeartRate=(PulsesCount*19)/2;
          HeartRatePrev=HeartRate;
          FlagInit=1;
        }
        else{
          HeartRate=(((PulsesCount*12.2)/2)+HeartRatePrev)/2;
          HeartRatePrev=HeartRate;
        }

        iniciotiempoContador=currentMillisContador;  
        PulsesCount=0;
                        
        rms=sqrt(rms/ContRms);
        IRMS=sqrt(IRMS/ContRms);
        R=log(rms)/log(IRMS);
        ContRms=0;
        rms=0;
        IRMS=0;              
      }    

      rms=rms+pow(s,2);
      IRMS=IRMS+pow(IRS,2);

      BpmFire = floor(HeartRate);
      SpO2Fire = floor(115-16*R);     
        
      if(SpO2Fire>100){
        SpO2Fire = 100;
      }
      if(SpO2Fire<0){
        SpO2Fire = 0;
      }   
      break;                           
    }   
  }  
}

void VerificarConexionFirebase(){
    
    unsigned long CurrentMillisCnxFire=millis(); 

    if(CurrentMillisCnxFire-PreviousMillisCnxFire1 >= Interval_Ctrl_Cnx_Fire1){
      PreviousMillisCnxFire1=CurrentMillisCnxFire;
      Firebase.setInt(firebaseData, pathConexion, 1);
    }    
}

//================================================================================================= FUNCIONES GENERALES
void ConexionWifi(){  //--------------------Va en el loop
  if (WiFi.status() != WL_CONNECTED){
    EstadoWifi=ESTADO_WIFI_DESCONECTADO;
           
  }else{
    EstadoWifi=ESTADO_WIFI_CONECTADO;
    VerificarConexionFirebase();
  }
}

void EstadoPosicionDedo(){
  if(particleSensor.getRed() < 70000){
    EstadoDedo=ESTADO_SIN_DEDO;
    
    PreviousMillisCalculo1=millis();
    PreviousMillisCalculo2=millis();
    StdC1 = 0;
    StdC2 = 0;
    FC1=0;
    FC2=0;

    PreviousMillisFire1 = millis();
    PreviousMillisFire2 = millis();
    PreviousMillisFire3 = millis();
    PreviousMillisFire4 = millis();
    FF1 = 0;
    FF2 = 0;
    StdF1 = 0;
    StdF2 = 0;
    StdF3 = 0;
    StdF4 = 0;           
    
    }else {
    EstadoDedo=ESTADO_CON_DEDO;    
  }
}

void ControlCalculo(){
  
 unsigned long CurrentMillisCalculo = millis();
       
    if(((CurrentMillisCalculo-PreviousMillisCalculo1) >= Interval_Control_Calculo1)){
      PreviousMillisCalculo1=CurrentMillisCalculo;
      StdC1 = 1;
    }
    if(((CurrentMillisCalculo-PreviousMillisCalculo2) >= Interval_Control_Calculo2)){
      PreviousMillisCalculo1=CurrentMillisCalculo;
      StdC2 = 1;
    }
    if((StdC1 == 1)&&(StdC2 == 0)) FC1=1;else FC1=0;
    if((StdC1 == 1)&&(StdC2 == 1)) FC2=1;else FC2=0;
    if(FC1==1) Std_Dedo_Cal=1;
    if(FC2==1) Std_Dedo_Cal=0;      
}

void ControlImpresionPantalla(){
  
 unsigned long CurrentMillisOledAv = millis();

  if(((CurrentMillisOledAv-PreviousMillisMed1) >= Interval_Oled_SW_MED_Std1)){
    PreviousMillisMed1=CurrentMillisOledAv;
    StdMed1=1;
  }
  if(((CurrentMillisOledAv-PreviousMillisMed2) >= Interval_Oled_SW_MED_Std2)){
    PreviousMillisMed2=CurrentMillisOledAv;
    StdMed2=1;   
  }  
  if(((CurrentMillisOledAv-PreviousMillisMed3) >= Interval_Oled_SW_MED_Std3)){
    PreviousMillisMed3=CurrentMillisOledAv;
    StdMed3=1;     
  }  
  if(((CurrentMillisOledAv-PreviousMillisMed4) >= Interval_Oled_SW_MED_Std4)){
    PreviousMillisMed4=CurrentMillisOledAv;
    StdMed4=1;    
  }  
  if(((CurrentMillisOledAv-PreviousMillisMed5) >= Interval_Oled_SW_MED_Std5)){
    PreviousMillisMed5=CurrentMillisOledAv;
    StdMed5=1;    
  }   

  if((StdMed1==1)&&(StdMed2==0)&&(StdMed3==0)&&(StdMed4==0)&&(StdMed5==0)){ 
    F1=1;
    F1CW=1;
  } else {
    F1=0;  //1%
    F1CW=0;
  }
  if((StdMed1==1)&&(StdMed2==1)&&(StdMed3==0)&&(StdMed4==0)&&(StdMed5==0)){ 
    F2=1;
    F2CW=1;
  }  else {
    F2=0;  //33%
    F2CW=0;
  }
  if((StdMed1==1)&&(StdMed2==1)&&(StdMed3==1)&&(StdMed4==0)&&(StdMed5==0)){ 
    F3=1;
    F3CW=1;
  }  else {
    F3=0;  //66%
    F3CW=0;
  }
  if((StdMed1==1)&&(StdMed2==1)&&(StdMed3==1)&&(StdMed4==1)&&(StdMed5==0)){ 
    F4=1;
    F4CW=1;
  }  else {
    F4=0;  //99%
    F4CW=0;
  }
  if((StdMed1==1)&&(StdMed2==1)&&(StdMed3==1)&&(StdMed4==1)&&(StdMed5==1)){ 
    F5=1;
    F5CW=1;
  }  else {
    F5=0;  // Muestra medida
    F5CW=0;
  }
}

void ControlEnvioDatosFirebase(){
           
    unsigned long CurrentMillisFire=millis(); 

    if(CurrentMillisFire-PreviousMillisFire1 >= Interval_Control_Fire1){
      PreviousMillisFire1=CurrentMillisFire;
      StdF1=1;
    }
    if(CurrentMillisFire-PreviousMillisFire2 >= Interval_Control_Fire2){
      PreviousMillisFire2=CurrentMillisFire;
      StdF2=1;
    }
    if(CurrentMillisFire-PreviousMillisFire3 >= Interval_Control_Fire3){
      PreviousMillisFire3=CurrentMillisFire;
      StdF3=1;
    }
    if(CurrentMillisFire-PreviousMillisFire4 >= Interval_Control_Fire4){
      PreviousMillisFire4=CurrentMillisFire;
      StdF4=1;
    }  

    if((StdF1==1)&&(StdF2==0)&&(StdF3==0)&&(StdF4==0)) FF1=1; else FF1=0;
    if((StdF1==1)&&(StdF2==1)&&(StdF3==1)&&(StdF4==0)) FF2=1; else FF2=0;      
}

void ImprimirDatos(){ //======================================================================== FUNCION 2. IMPRIMIR DATOS
  switch(EstadoWifi){
    case ESTADO_WIFI_CONECTADO:
        MedicionConWifi();
        EstadoPosicionDedo();
    break;
    case ESTADO_WIFI_DESCONECTADO:
        MedicionSinWifi();
        EstadoPosicionDedo();
    break;
    default:
        break;            
  }
}

void MedicionConWifi(){   //-------------------------------------------- Medición CON WiFi
  switch (EstadoDedo){
    case ESTADO_SIN_DEDO:
        ImprimirColocarDedoCW();        
        break;
    case ESTADO_CON_DEDO:
        Calcular();
        EnviarDatosFirebase();
        MostrarMedicionConWifi();
        break;       
    default:
        break;        
  }
}

void ImprimirColocarDedoCW(){
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(34,0);
        oled.println("SIGNAL-MED");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(110,0);
        oled.println(".il");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(19,29);
        oled.println("Coloque su dedo");
        oled.drawLine(0, 10, 128, 10, 1);
        oled.display(); 

        StdMed1=0;
        StdMed2=0;
        StdMed3=0;
        StdMed4=0;
        StdMed5=0;

        F1CW=0;
        F2CW=0;
        F3CW=0;
        F4CW=0;
        F5CW=0;
        
        PreviousMillisMed1=millis();
        PreviousMillisMed2=millis();
        PreviousMillisMed3=millis();
        PreviousMillisMed4=millis();
        PreviousMillisMed5=millis();
}

void MostrarMedicionConWifi(){

  ControlImpresionPantalla();    
 
  if(F1CW==1){
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(34,0);
        oled.println("SIGNAL-MED");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(110,0);
        oled.println(".il");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(6,29);
        oled.println("Realizando medicion");
        oled.setCursor(0,45);
        oled.println("=");
        oled.drawLine(0, 10, 128, 10, 1);
        oled.display();      
  }
  if(F2CW==1){
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(34,0);
          oled.println("SIGNAL-MED");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(110,0);
          oled.println(".il");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(6,29);
          oled.println("Realizando medicion");
          oled.setCursor(0,45);
          oled.println("=======");
          oled.drawLine(0, 10, 128, 10, 1);
          oled.display();  
  }
  if(F3CW==1){
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(34,0);
          oled.println("SIGNAL-MED");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(110,0);
          oled.println(".il");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(6,29);
          oled.println("Realizando medicion");
          oled.setCursor(0,45);
          oled.println("==============");
          oled.drawLine(0, 10, 128, 10, 1);
          oled.display();     
  }
  if(F4CW==1){
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(34,0);
          oled.println("SIGNAL-MED");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(110,0);
          oled.println(".il");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(6,29);
          oled.println("Realizando medicion");
          oled.setCursor(0,45);
          oled.println("=====================");
          oled.drawLine(0, 10, 128, 10, 1);
          oled.display(); 
  }      
  if(F5CW==1){
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(34,0);
        oled.println("SIGNAL-MED");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(110,0);
        oled.println(".il");
        oled.drawLine(0, 10, 128, 10, 1);
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(0,20);
        oled.println("Pulso:");
        oled.setCursor(40,20);
        oled.println(BpmFire);
        oled.setCursor(63,20);
        oled.println("bpm");
        oled.setCursor(0, 37);
        oled.println("SpO2:");
        oled.setCursor(33,37);
        oled.println(SpO2Fire);
        oled.setCursor(56,37);
        oled.println("%");
        oled.display();   
  }
}

void MedicionSinWifi(){   //-------------------------------------------- Medición SIN WiFi
  switch (EstadoDedo){
    case ESTADO_SIN_DEDO:
        ImprimirColocarDedoSW();        
        break;
    case ESTADO_CON_DEDO:
        Calcular();
        MostrarMedicionSinWifi();
        break;       
    default:
        break;        
  }
}

void ImprimirColocarDedoSW(){
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(34,0);
        oled.println("SIGNAL-MED");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(110,0);
        oled.println("...");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(19,29);
        oled.println("Coloque su dedo");
        oled.drawLine(0, 10, 128, 10, 1);
        oled.display(); 

        StdMed1=0;
        StdMed2=0;
        StdMed3=0;
        StdMed4=0;
        StdMed5=0;

        F1=0;
        F2=0;
        F3=0;
        F4=0;
        F5=0;
        
        PreviousMillisMed1=millis();
        PreviousMillisMed2=millis();
        PreviousMillisMed3=millis();
        PreviousMillisMed4=millis();
        PreviousMillisMed5=millis();
}

void MostrarMedicionSinWifi(){

  ControlImpresionPantalla();    
 
  if(F1==1){
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(34,0);
        oled.println("SIGNAL-MED");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(110,0);
        oled.println("...");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(6,29);
        oled.println("Realizando medicion");
        oled.setCursor(0,45);
        oled.println("=");
        oled.drawLine(0, 10, 128, 10, 1);
        oled.display();      
  }
  if(F2==1){
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(34,0);
          oled.println("SIGNAL-MED");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(110,0);
          oled.println("...");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(6,29);
          oled.println("Realizando medicion");
          oled.setCursor(0,45);
          oled.println("=======");
          oled.drawLine(0, 10, 128, 10, 1);
          oled.display();  
  }
  if(F3==1){
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(34,0);
          oled.println("SIGNAL-MED");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(110,0);
          oled.println("...");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(6,29);
          oled.println("Realizando medicion");
          oled.setCursor(0,45);
          oled.println("==============");
          oled.drawLine(0, 10, 128, 10, 1);
          oled.display();     
  }
  if(F4==1){
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(34,0);
          oled.println("SIGNAL-MED");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(110,0);
          oled.println("...");
          oled.setTextSize(1);
          oled.setTextColor(1);
          oled.setCursor(6,29);
          oled.println("Realizando medicion");
          oled.setCursor(0,45);
          oled.println("=====================");
          oled.drawLine(0, 10, 128, 10, 1);
          oled.display(); 
  }      
  if(F5==1){
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(34,0);
        oled.println("SIGNAL-MED");
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(110,0);
        oled.println("...");
        oled.drawLine(0, 10, 128, 10, 1);
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.setCursor(0,20);
        oled.println("Pulso:");
        oled.setCursor(40,20);
        oled.println(BpmFire);
        oled.setCursor(63,20);
        oled.println("bpm");
        oled.setCursor(0, 37);
        oled.println("SpO2:");
        oled.setCursor(33,37);
        oled.println(SpO2Fire);
        oled.setCursor(56,37);
        oled.println("%");
        oled.display();   
  }
}

void EnviarDatosFirebase(){ //------------------------------------------------------ 3) FUNCION ENVÍO A FIREBASE
  
  ControlEnvioDatosFirebase();
  
  if(FF1==1){
      Firebase.setInt(firebaseData, pathBPM, 0);
      Firebase.setInt(firebaseData, pathSpO2, 0);  
      //Firebase.setInt(firebaseData, pathInicioMedicion, 0);    
          
  }
    if(FF2==1){
      Firebase.setInt(firebaseData, pathBPM, BpmFire);
      Firebase.setInt(firebaseData, pathSpO2, SpO2Fire);  
  }
}

void loop() {
  ConexionWifi();
  ImprimirDatos();  
}
