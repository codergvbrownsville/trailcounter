#include <SD.h>                         //importing sd library - for use with memory card - i/o
#include <SPI.h>                        //importing spi (serial peripheral interface) library - for use with memory card - to speak with writer

const int counterLed = 8;               //initializing state of counterLed for use as pin number later
const int programLed = 6;               //initializing state of programLed for use as pin number later
const int pushButton = 7;               //initializing state of pushButton for use as pin number later
int programLedState = LOW;              //initializing state of the programLedState (on/off) led
int buttonState;                        //declaring the buttonState variable
int lastButtonState = LOW;              //initializing state of the lastButtonState
int reading = LOW;                      //initializing state of reading
long lastDebounceTime = 0;              //last time the pushbutton was toggled - Used to ignore extra, inadvertent, button pushes
long debounceDelay = 50;                //debounce time threshold for pushbutton - Used to ignore extra, inadvertent, button pushes
const int chipSelect = 10;              //From SPI library; initializing state of chipSelect for use as a pin number; used to check if SD card present
long cm = 0;                            //initializing state of cm
int incomingByte = 0;                   //initializing state of incomingByte - this is the initiation of the program by pressing 1
int state = 0;                          //initializing state of state; this is determined by counter and sensor?
int counter = 0;                        //initializing state of counter; keeps track of the number of counts
int sensor = 0;                         //initializing state of sensor; this is the digital distance value from the Lidar
unsigned long timeSeconds = 0;          //initializing state of timeSeconds; unisigned is highest storage for a variable
const int HIGH_TRIGGER = 50;            //threshold distance (cm) that will count as a hit
const int TRIGGER_DELAY = 50;           //length of time (milliseconds) that trigger does not activate to prevent false hits

File sensorData;                        //variable for working with file object

void setup() {
    //pinMode(sensorPin, OUTPUT);
    pinMode(10,OUTPUT);               //reserved for library use, not used in our program
    pinMode(counterLed, OUTPUT);

    pinMode(pushButton, INPUT);
    pinMode(programLed, OUTPUT);

    digitalWrite(programLed, programLedState);    // set initial programLED state

    Serial.begin(9600);

    //-------------------

 pinMode(2, OUTPUT); // Set pin 2 as trigger pin
 digitalWrite(2, LOW); // Set trigger LOW for continuous read
 pinMode(3, INPUT); // Set pin 3 as monitor pin

//-------------------

    state = 0;
    while(!Serial) {
      ;
    }

    Serial.println("Initializing SD Card...");

    if(!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      digitalWrite(counterLed, HIGH);
      return;
    }

    Serial.println("Card initialized.");

    Serial.println();
    Serial.println("Ready...");
    Serial.println("Hold button for 1 second or type 0 and hit enter to begin counter.");
    Serial.println();
}

void loop() {
  reading = digitalRead(pushButton);
  delay(100);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();                          // reset the debouncing timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {   //filters false pushbutton signal
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {                        // only toggle the LED if the new button state is HIGH
        programLedState = !programLedState;
      }
    }
  }

  digitalWrite(programLed, programLedState);            // set the LED:
  lastButtonState = reading;                            //store reading into lastButtonState

  switch (state) {

    case 0:

      incomingByte = Serial.read();

      if (incomingByte == '0' || programLedState == HIGH)
      {
        timeSeconds = (millis() / 1000);
        programLedState = HIGH;
        digitalWrite(counterLed, LOW);

        state = 1;
        Serial.println("starting counter");
        sensorData = SD.open("ctrData.txt", FILE_WRITE);  //opens txt file to write data

        if(sensorData)  {     //once file opens successfully run this
          Serial.print("Starting counter on ");
          Serial.println(timeSeconds);

          //sensorData.print("Starting counter on ");
          sensorData.println(timeSeconds);

          sensorData.close();
        }
      }

      break;
    case 1:

      sensor = getSensorData();
                                      Serial.print("case 1 sensor reading: ");
                                      Serial.println(sensor);
                                      Serial.println();

      if (HIGH_TRIGGER <= sensor)
      {
        state = 2; //reading
                                      Serial.println("high trigger < sensor TRUE");
                                      Serial.println("state 2");
                                      Serial.println();
      }
      break;

    case 2:

      delay(TRIGGER_DELAY);
      sensor = getSensorData();
                                      Serial.print("case 2 sensor reading: ");
                                      Serial.println(sensor);
                                      Serial.println();

      if (HIGH_TRIGGER > sensor)
      {
        state = 3;
                                      Serial.println("high trigger > sensor TRUE");
                                      Serial.println("state 3");
                                      Serial.println();
      }

      break;
    case 3:

      delay(TRIGGER_DELAY);
      sensor = getSensorData();
                                      Serial.print("case 3 sensor reading: ");
                                      Serial.println(sensor);
                                      Serial.println();

      if (HIGH_TRIGGER <= sensor)
      {
        state = 4;

        counter++;
                                      Serial.println("high trigger < sensor TRUE");
                                      Serial.println("state 4");
                                      Serial.println();
      }

      break;

    case 4:

      timeSeconds = (millis() / 1000);

      sensorData = SD.open("ctrData.txt", FILE_WRITE);  //opens txt file to write data

      if(sensorData)  {     //once file opens successfully run this
        Serial.print(counter);
        Serial.print(" ");
        Serial.println(timeSeconds);

        sensorData.println(timeSeconds);                //writes data to file
        sensorData.close();                             //closes file

        digitalWrite(counterLed, HIGH);                 // turns the counter LED on during detection
        delay(50);                                      // delay allows time for led change
        digitalWrite(counterLed, LOW);                  // turns the counterLED off
        delay(50);                                      // delay allows time for led change
      }

      state = 1;
      break;
    default:

      state = 0;
  }
}



//program functions
int getSensorData() {
    /*
     i=analogRead(sensorPin);


    //valFront     = (6762.00/(i-9))-4;
    cm     = 28250/(i-229.5);     //Then, we have a linear equation of digital value and distance.
                                          //(512-sensorValue)/(1/100-1/distance)=(512-286)/(0.01-0.002)
                                          //=> distance=28250/(sensorValue-229.5);


    Serial.print("sensor Front: ");
    Serial.println(cm);           //print val1 with 1 decimal point accuracy
*/
cm = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  if(cm != 0)
  {
    cm = cm / 10; // 10usec = 1 cm of distance
    Serial.println(cm); // Print the distance

}
}
