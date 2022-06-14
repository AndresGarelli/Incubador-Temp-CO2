//CO2 sensor https://www.co2meter.com/products/cozir-wrv-20-percent-co2-sensor
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <cozir.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP_Mail_Client.h>
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "xxxxxxxxxxxx@gmail.com"
#define AUTHOR_PASSWORD "xxxxxxxxxx"

#define WIFISSID "xxxxxx" /* Put here your Wi-Fi SSID */
#define PASSWORD ""

//Las resistencias para calentar están controladas por un PID
//Variables para el PID
//Constantes para PID
double Setpoint, Input, Output;

double Kp = 100, Ki = 10, Kd = 1; //ESTA ES MEJOR AUN. PROBAR CON SAMPLE TIME 3SEG
//double Kp = 800, Ki = 20, Kd = 900;
//double Kp = 100, Ki = 10, Kd = 1;
PID myPIDcalor(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);


bool mailEncendido = true;
bool mailAlarma = false;
bool mailAlarmaEnviado = false;
unsigned long bajoTempCO2ON;
bool bajoTempCO2 = true;

String asunto;
float temp1;
bool todoBien = true;
unsigned long previousMillis = 0;       //para refresh serialport
unsigned long previousMillisMedicion = 0;   //esto es para determinar el tiempo entre medidas de sensor
unsigned long prevLoad = 0;
unsigned long prevLoad2 = 0;
bool puertaAbierta = false;
bool alarmEnabled = true;
unsigned long alarmOFF = 0;  //
bool recienEncendido = true;

//variables para botón reset
int botonState = HIGH;
int prevBotonState = HIGH;
bool ignoreUp = false;
unsigned long btnUpTime;
unsigned long btnDnTime;

unsigned long lcdOnTime;

float gap;           //variable para la alarma, mide la diferencia con la temperaruta deseada
String estado;
const int pinRelayCalor = 12;   //es D6 en nodemcurelay resistencias
const int buzz = 13;            //buzzer. D7 in nodemcu
const int buzzLED = 15;         //nodemcu D8 led testigo de alarma
const int releCO2 = 4; // D2 queda libre para CO2
const int puerta = 16; //pin sensor puerta abierta, D0
const int resetPin = 2; //pin para botón reset alarma, D4
//const int LED = 2; //led azul para boot
int trial = 0;
int ONtime; //tiempo de apertura de válvula CO2
bool abrir = false;
int WindowSizeCO2 = 5000;                                      //windowsizecalor es el tiempo maximo que puede estar encendida la resistencia en mseg
unsigned long windowStartTime = millis();
float d;

/////////////////////////////////////////////////////////////////////


////CODIGO DS18B20
// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 14;   ///////////////////D5
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

// Variables con las direcciones únicas de los 4 sensores DS18B20
DeviceAddress Lorena = {address of the DS18B20 sensor};
String direccion = "Incubador";
String direccion2 = "IncubadorCO2";
////FIN DS18B20

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

COZIR czr(&Serial);


void setup() {
  analogWriteFreq(200);
  pinMode(pinRelayCalor, OUTPUT);
  digitalWrite(pinRelayCalor, LOW);
  pinMode (puerta, INPUT_PULLDOWN_16);//atención, del pin 0-15 se puede poner pullup. sólo 16 acepta pulldown
  pinMode (buzz, OUTPUT);
  pinMode (buzzLED, OUTPUT);
  digitalWrite(buzzLED, LOW);
  pinMode (resetPin, INPUT_PULLUP);
  pinMode(releCO2, OUTPUT);
  digitalWrite(releCO2, LOW);
  // Inicializar el LCD
  Wire.begin(0, 5); //inicia biblioteca wire, asigna SDA a pin0 (D3) y SCL a pin 5 (D1);
  lcd.init();

  //Encender la luz de fondo.
  lcd.backlight();

  timeClient.begin();
  timeClient.setTimeOffset(-10800);

  // Escribimos el Mensaje en el LCD.
  WiFi.begin(WIFISSID, PASSWORD);
  lcd.clear();
  lcd.print("VIRUS 1.7");
  lcd.setCursor(0, 1);
  lcd.print("Red INIBIBB");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("buscando red");
  while (WiFi.status() != WL_CONNECTED & trial < 10)
  { lcd.setCursor(trial, 1);
    lcd.print(".");
    delay(500);
    trial++;
  }

  Output = 0;

  //DS18B20
  // Iniciamos el bus 1-Wire
  sensorDS18B20.begin();

  Serial.begin(9600);
  czr.init();

  int contar = sensorDS18B20.getDeviceCount();
  while (contar != 1) {
    lcd.setCursor(0, 0);
    lcd.print("ERROR SENSOR");
    lcd.setCursor(0, 1);
    lcd.print(contar);
    lcd.print(" sensores");
    delay(1000);
  }

  lcd.clear();
  lcd.print("VIRUS 1.7");
  lcd.setCursor(0, 1);
  lcd.print(sensorDS18B20.getDeviceCount());
  lcd.print(" sensor OK!");
  delay(1000);

  //FIN DS18B20

  // MODIFICAR ESTE VALOR PARA CAMBIAR EL SETPOINT EN GRADOS CELCIUS//////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Setpoint = 37.0;
  //////////////////////////////////////////////////////////////////////////7

  //CAMBIADA. antes era Setpoint = (Setdeseado - 0.10);
  //indica el rango de tiempo en mseg que pueden estar encendidas las resistencias
  myPIDcalor.SetOutputLimits(0, 1023); //en nodemcu 100% corresponde a 1023, en arduino 255
  myPIDcalor.SetSampleTime(2000);        // tiempo entre cálculos de PID

  myPIDcalor.SetMode(AUTOMATIC);
  lcdOnTime = millis();
  lcd.clear();



}

void loop() {

  /////control de estado de la puerta

  if (digitalRead(puerta) == HIGH)
  { puertaAbierta = false;
  } else (puertaAbierta = true);

  /////////CONFIGURACIÓN BOTÓN RESET  /////////////

  botonState = digitalRead(resetPin);

  // Test for button pressed and store the down time
  if (botonState == LOW && prevBotonState == HIGH && (millis() - btnUpTime) > 50)
  {
    btnDnTime = millis();
  }

  // Test for button release and store the up time
  if (botonState == HIGH && prevBotonState == LOW && (millis() - btnDnTime) > 50) {
    btnUpTime = millis();
    if (ignoreUp == false) {
      alarmEnabled = false;
      alarmOFF = millis();
      lcdOnTime = millis();
      lcd.backlight();
    } else {
      ignoreUp = false;
    }
  }
  prevBotonState = botonState;

  if (millis() - lcdOnTime > 120000) {
    lcd.noBacklight();
  }
  //////////Habilita alarma luego de 15 minutos de puerta cerrada o reseteo alarma

  if (millis() - alarmOFF > 900000) (alarmEnabled = true);

  ///ALARMA////
  gap = fabs(Setpoint - temp1);

  if  (millis() > 900000) {
    if  (gap > 0.5 || d < 4.8) {
      digitalWrite (buzzLED, HIGH);
      lcdOnTime = millis();
      lcd.backlight();
    } else {
      digitalWrite (buzzLED, LOW);
    }

    if  (gap > 0.5 || d < 4.8) {
      if (alarmEnabled == true && puertaAbierta == false) {
        analogWrite(buzz, 200);
      } else {
        digitalWrite(buzz, LOW);
      }
    } else {
      digitalWrite(buzz, LOW);
    }

  }

  if (puertaAbierta == true) {
    //myPIDcalor.SetMode(MANUAL);
    //Output = 0;
    //digitalWrite(pinRelayCalor,LOW);
    digitalWrite(buzz, LOW);
    alarmEnabled = false;
    alarmOFF = millis();
  }


  if (millis() - previousMillisMedicion >= 2000) {
    //previousMillisMedicion = millis();                 /// con puerta cerrada inicia PID, pero aunque la puerta esté abierta, igual mide la temperatura cada 2 seg
    sensorDS18B20.requestTemperatures();
    uint32_t c = czr.CO2();
    c = c * 10;
    d = float(c) / 10000;  //concentración CO2 en porcentaje

    float temp =  sensorDS18B20.getTempC(Lorena);
    //float temp =  sensorDS18B20.getTempCByIndex(0);

    if (temp > 4 && temp < 65 ) {
      previousMillisMedicion = millis();
      temp1 = temp;
      myPIDcalor.SetMode(AUTOMATIC);
      todoBien = true;
    } else {   //comentar en programa definitivo
      todoBien = false;
      myPIDcalor.SetMode(MANUAL);
      Output = 0;
      temp1 = temp;
    }
  }


  Input = temp1;
  myPIDcalor.Compute();
  yield();
  analogWrite(pinRelayCalor, Output);

  if (d < 4.8) {
    ONtime = WindowSizeCO2;
  } else {
    ONtime = 1000;
  }

  if (d < 4.95) {
    abrir = 1;
  } else if (d >= 5.0000) {
    abrir = 0;
  }


  controlCO2();

  if (millis() - prevLoad >= 2500) {
    prevLoad = millis();
    // Ubicamos el cursor en la primera posición(columna:0) de la segunda línea(fila:1)
    if (todoBien == true && puertaAbierta == false) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("S:");
      lcd.print(Setpoint, 1);
      lcd.print("| CO2|RED");
      if (WiFi.status() == WL_CONNECTED) {
        lcd.print("+");
      } else {
        lcd.print("-");
      }
      //lcd.print((Output * 100 / 1023), 0);
      // lcd.print("%");
      lcd.setCursor(12, 0);
      //lcd.print("|RED");
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(temp1, 1);
      lcd.print("| ");
      lcd.print(d, 1);
      lcd.print("|");
      lcd.print((Output * 100 / 1023), 0);
      lcd.print("%");

    } else if  (todoBien == true && puertaAbierta == true) {
      lcdOnTime = millis();
      lcd.backlight();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Puerta Abierta!");
      lcd.setCursor(0, 1);
      lcd.print(" OUT ");
      lcd.print((Output * 100 / 1023), 0);
      lcd.print("%");
      lcd.print(" T:");
      lcd.print(temp1, 1);
    } else if (todoBien == false) {
      lcdOnTime = millis();
      lcd.backlight();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ERROR sensor!");
      lcd.setCursor(0, 1);
      lcd.print(" OUT ");
      lcd.print((Output * 100 / 1023), 0);
      lcd.print("%");
      lcd.print(" T:");
      lcd.print(temp1, 1);
    }
  }

  if (millis() - prevLoad2 >= 30000) {
    enviarTempTW();
    enviarTempCO2();
    prevLoad2 = millis();
  }

  if (recienEncendido == true && WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    asunto = timeClient.getFormattedTime() + " - Inicio incubador Virus - temp: "  + temp1 + " - CO2: " + d;
    mailEncendido = true;
    enviarEmail();
  }

  if  (millis() > 900000) { //cambiar a 900000 o 15 minutos
    if (temp1 < Setpoint - 0.5 || d <= 4.50) {
      if (bajoTempCO2 == false) {
        bajoTempCO2ON = millis();
        bajoTempCO2 = true;
      }
    } else {
      bajoTempCO2ON = millis();
      bajoTempCO2 = false;
    }
  }

  if (millis() - bajoTempCO2ON > 1800000 && mailAlarmaEnviado == false) {
    if (WiFi.status() == WL_CONNECTED) {
      timeClient.update();
      asunto = timeClient.getFormattedTime() + " - error incubador Virus - temp: "  + temp1 + " - CO2: " + d;
      mailAlarma = true;
      enviarEmail();
    }
  }

}


void enviarTempTW() {
  String pedido = "http://xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx?sensor=";
  //Serial.println(direccion);
  pedido = pedido + direccion;
  pedido = pedido + "&temperatura=";
  pedido = pedido + String(temp1);

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(pedido);
    int httpCode = http.GET();

    /*
      if (httpCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpCode);
      Serial.println("");
       }
       else {
         Serial.print("Error code: ");
         Serial.println(httpCode);
         Serial.println("");
             }*/
    // Free resources
    http.end();
  }
  else {
    //    Serial.println("WiFi Disconnected");
  }
}

void enviarTempCO2() {
  String pedido = "http://xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxp?sensor=";
  //Serial.println(direccion);
  pedido = pedido + direccion2;
  pedido = pedido + "&temperatura=";
  pedido = pedido + String(d);

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(pedido);
    int httpCode = http.GET();

    /*
      if (httpCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpCode);
      Serial.println("");
       }
       else {
         Serial.print("Error code: ");
         Serial.println(httpCode);
         Serial.println("");
             }*/
    // Free resources
    http.end();
  }
  else {
    //    Serial.println("WiFi Disconnected");
  }
}

void controlCO2() {
  if (puertaAbierta == false && abrir == 1) {
    if (millis() - windowStartTime > WindowSizeCO2) {  //time to shift the Relay Window
      windowStartTime = millis();
    }
    if (ONtime > millis() - windowStartTime) {
      digitalWrite(releCO2, HIGH);
    } else {
      digitalWrite(releCO2, LOW);
    }
  } else {
    digitalWrite(releCO2, LOW);
  }
}

void enviarEmail() {
  if (WiFi.status() == WL_CONNECTED) {
    //digitalWrite(LED, LOW);
    timeClient.update();
    SMTPSession smtp;
    smtp.debug(0);

    /* Declare the session config data */
    ESP_Mail_Session session;

    /* Set the session config */
    session.server.host_name = SMTP_HOST;
    session.server.port = SMTP_PORT;
    session.login.email = AUTHOR_EMAIL;
    session.login.password = AUTHOR_PASSWORD;
    //session.login.user_domain = "mydomain.net";

    /* Declare the message class */
    SMTP_Message message;

    /* Set the message headers */
    message.sender.name = "Incubador Virus";
    message.sender.email = AUTHOR_EMAIL;
    message.subject = asunto.c_str(); //agregué .c_str() porque no podía convertir String a char*
    message.addRecipient("Lorena", "xxxxxxxxxxxxxxxx@criba.edu.ar");
    message.addRecipient("Vicky", "xxxxxxxxxxxxxx@criba.edu.ar");

    //message.text.content = "This is simple plain text message";
    message.text.charSet = "us-ascii";
    message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
    message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_high;
    message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

    /* Set the custom message header */
    //message.addHeader("Message-ID: <abcde.fghij@gmail.com>");

    /* Connect to server with the session config */
    if (!smtp.connect(&session))
      return;
    // digitalWrite(LED, HIGH);
    /* Start sending Email and close the session */
    if (!MailClient.sendMail(&smtp, &message)) {
      //Serial.println("Error sending Email, " + smtp.errorReason());

    } else {
      if (mailEncendido == true) {
        recienEncendido = false;
        mailEncendido = false;
      }
      if (mailAlarma == true) {
        mailAlarmaEnviado = true;
        mailAlarma = false;
      }

    }
  }
}
