
#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DigitalIO.h>
#include <Bounce2.h>
#include <avr/wdt.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define MAX_ATTACHED_DS18B20 16

#define RADIO_RESET_DELAY_TIME 20 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения

#define NODE_ID 88


#define CHILD_ID_DOOR1 1
#define CHILD_ID_DOOR2 2
#define CHILD_ID_MOTION 3

#define CHILD_ID_TEMPERATURE 4  

#define REBOOT_CHILD_ID 100
#define DISABLE_MOTION_SENSOR_CHILD_ID 101
#define RECHECK_SENSOR_VALUES          102



#define TEMPERATURE_SENSOR_DIGITAL_PIN 6
#define DOOR1_SENSOR_DIGITAL_PIN 3
#define DOOR2_SENSOR_DIGITAL_PIN 4
#define MOTION_SENSOR_DIGITAL_PIN 5
#define LED_DIGITAL_PIN 7


#define NUMBER_OF_SWITCHES 2




boolean metric = true;          // Celcius or fahrenheid
float lastTemp = -1;
long previousTempMillis = 0;        // last time the sensors are updated
long TempsensorInterval = 60000;     // interval at which we will take a measurement ( 30 seconds)
int oldDebouncerState=-1;
boolean lastMotion=false;
boolean oldValue_switch1=-1;
boolean oldValue_switch2=-1;

unsigned long previousMSMillis=0;
unsigned long MSsensorInterval=60000;

boolean boolMotionSensorDisabled = false;
boolean boolRecheckSensorValues = false;


boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

OneWire oneWire(TEMPERATURE_SENSOR_DIGITAL_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

Bounce debouncer[NUMBER_OF_SWITCHES];

MySensor gw;

MyMessage TempMsg(CHILD_ID_TEMPERATURE, V_TEMP);
MyMessage DoorMsg1(CHILD_ID_DOOR1, V_TRIPPED);
MyMessage DoorMsg2(CHILD_ID_DOOR2, V_TRIPPED);
MyMessage MotionMsg(CHILD_ID_MOTION, V_TRIPPED);
MyMessage MotionStateMsg(DISABLE_MOTION_SENSOR_CHILD_ID, V_ARMED);


void setup() {
 
  
   //initialize led
   //pinMode(LED_DIGITAL_PIN,OUTPUT);
   digitalWrite(LED_DIGITAL_PIN,HIGH);
   
  // requestTemperatures() will not block current thread
 // sensors.setWaitForConversion(true);

      Serial.begin(115200);
    Serial.println("Begin setup");
    // Initialize library and add callback for incoming messages
    gw.begin(incomingMessage, NODE_ID, false);
      gw.wait(RADIO_RESET_DELAY_TIME);


  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Country home entrance sensor", "1.0");
      gw.wait(RADIO_RESET_DELAY_TIME);

   metric = gw.getConfig().isMetric;

  // Present all sensors to controller
     gw.present(CHILD_ID_TEMPERATURE, S_TEMP);
        gw.wait(RADIO_RESET_DELAY_TIME);     


          // Fetch temperatures from Dallas sensors
          sensors.requestTemperatures(); 
          float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.;
 
  // Setup the button
  pinMode(DOOR1_SENSOR_DIGITAL_PIN,INPUT);
  pinMode(DOOR2_SENSOR_DIGITAL_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(DOOR1_SENSOR_DIGITAL_PIN,HIGH);
  digitalWrite(DOOR2_SENSOR_DIGITAL_PIN,HIGH);
  

  // After setting up the button, setup debouncer
  debouncer[0] = Bounce();
  debouncer[0].attach(DOOR1_SENSOR_DIGITAL_PIN);
  debouncer[0].interval(5);
  gw.present(CHILD_ID_DOOR1, S_DOOR);  
        gw.wait(RADIO_RESET_DELAY_TIME);     

  gw.wait(250);
  
  debouncer[1] = Bounce();
  debouncer[1].attach(DOOR2_SENSOR_DIGITAL_PIN);
  debouncer[1].interval(5);
  gw.present(CHILD_ID_DOOR2, S_DOOR);  
        gw.wait(RADIO_RESET_DELAY_TIME);     

  gw.wait(250);
  


  
  
  
  //Motion sensor
  pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);      // sets the motion sensor digital pin as input
  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_MOTION, S_MOTION);
        gw.wait(RADIO_RESET_DELAY_TIME);   
  

//reboot sensor command
     gw.present(REBOOT_CHILD_ID, S_BINARY); 
         gw.wait(RADIO_RESET_DELAY_TIME);    

//disable-enable motion sensor
     gw.present(DISABLE_MOTION_SENSOR_CHILD_ID, S_MOTION); 
        gw.wait(RADIO_RESET_DELAY_TIME);   

//reget sensor values
  gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 
        gw.wait(RADIO_RESET_DELAY_TIME);   
        
/***************   Send initial state of sensors to gateway  ****************/

     debouncer[0].update();
  // Get the update value
//    int value = debouncer.read();
    boolean value;
    value = debouncer[0].read() == HIGH;
     gw.send(DoorMsg1.set(value ? "1" : "0"));
        gw.wait(RADIO_RESET_DELAY_TIME);   
  
    debouncer[1].update();
    boolean value2;
    value2 = debouncer[1].read() == HIGH;
     gw.send(DoorMsg2.set(value2 ? "1" : "0"));   
        gw.wait(RADIO_RESET_DELAY_TIME);   
  
  
  boolean motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
  gw.send(MotionMsg.set(motion ? "1" : "0" ));  // Send motion value to gw
  
// ***************   End send initial state of sensors to gateway  ****************/

    //Enable watchdog timer
    wdt_enable(WDTO_8S);
    
    Serial.println("End setup");  

   digitalWrite(LED_DIGITAL_PIN,LOW);

}

void loop() {
  // put your main code here, to run repeatedly:


 // Read digital motion value
 if ( !boolMotionSensorDisabled )
 {
  boolean motion = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
   if (lastMotion != motion || boolRecheckSensorValues) {
    Serial.println("Motion detected");
  lastMotion = motion;     

    //Отсылаем состояние сенсора с подтверждением получения
    iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
      gw.send(MotionMsg.set(motion ? "1" : "0" ), true);  // Send motion value to gw
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;

   
     if (motion == 1)    digitalWrite(LED_DIGITAL_PIN,HIGH);
          else        digitalWrite(LED_DIGITAL_PIN,LOW);
  }

  }

checkTemp();



  //check doors state
    debouncer[0].update();
  // Get the update value
//    int value = debouncer.read();
    boolean value;
    value = debouncer[0].read() == HIGH;
 
  if (value != oldValue_switch1 || boolRecheckSensorValues) {

  //Отсылаем состояние сенсора с подтверждением получения
  iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
      gw.send(DoorMsg1.set(value ? "1" : "0"), true);  // Send motion value to gw
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;

     oldValue_switch1 = value;
         Serial.print("Door1: ");
        Serial.println(value);
     if (value == 1)    digitalWrite(LED_DIGITAL_PIN,HIGH);
          else        digitalWrite(LED_DIGITAL_PIN,LOW);        
  }
  
  
    //check doors state
    debouncer[1].update();
  // Get the update value
//    int value = debouncer.read();
    boolean value2;
    value2 = debouncer[1].read() == HIGH;
 
  if (value2 != oldValue_switch2 || boolRecheckSensorValues) {

  //Отсылаем состояние сенсора с подтверждением получения
  iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
      gw.send(DoorMsg2.set(value2 ? "1" : "0"), true);   // Send motion value to gw
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }

    gotAck = false;

       
     oldValue_switch2 = value2;
        Serial.print("Door2: ");
        Serial.println(value2);
     if (value2 == 1)    digitalWrite(LED_DIGITAL_PIN,HIGH);
          else        digitalWrite(LED_DIGITAL_PIN,LOW);           
  }
  

reportMotionSensorState();

    if (boolRecheckSensorValues)
      {
       boolRecheckSensorValues = false;
      }

    // Alway process incoming messages whenever possible
    gw.process();
    
    //reset watchdog timer
        wdt_reset();
}


void checkTemp()
{

    unsigned long currentTempMillis = millis();
    if(currentTempMillis - previousTempMillis > TempsensorInterval || boolRecheckSensorValues) {
        // Save the current millis
        previousTempMillis = currentTempMillis;
        // take action here:

          // Fetch temperatures from Dallas sensors
          sensors.requestTemperatures();
    //float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0))) / 10.;
       // float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)) * 10.)) / 10.;
        float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.;

        Serial.print("Temp: ");
        Serial.println(temperature);
        if (temperature != lastTemp || boolRecheckSensorValues) {
            gw.send(TempMsg.set(temperature,1));
            lastTemp = temperature;
        } 
        
        
    }    

  
}


void reportMotionSensorState()
{

    unsigned long currentMSMillis = millis();
    if(currentMSMillis - previousMSMillis > MSsensorInterval  || boolRecheckSensorValues) {
        // Save the current millis
        previousMSMillis = currentMSMillis;
        // take action here:

       gw.send(MotionStateMsg.set(boolMotionSensorDisabled ? "1" : "0" ));  
        
    }    

  
}



void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

  if (message.isAck())
  {
    gotAck = true;
    return;
  }


    if ( message.sensor == REBOOT_CHILD_ID ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }

     if ( message.sensor == DISABLE_MOTION_SENSOR_CHILD_ID ) {
         
         if (message.getBool() == true)
         {
            boolMotionSensorDisabled = true;
         }
         else
         {
            boolMotionSensorDisabled = false;
         }

         gw.send(MotionStateMsg.set(boolMotionSensorDisabled ? "1" : "0" )); 

     }
     
    if ( message.sensor == RECHECK_SENSOR_VALUES ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
         }

     }       

        return;      
} 


