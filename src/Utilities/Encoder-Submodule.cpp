#include <Arduino.h>
#include <Wire.h>

#define ADDRESS 0x41 // slave address
byte error;
int lineNum = 0;
int size = 5;
uint8_t message = 0x6; 
bool sent = false;
bool received = false;

void checkError(byte error);

void setup(){
    Wire.begin(21, 22, 400000);
    //Wire.setClock(400000); 
    Serial.begin(115200);
    Wire.setTimeOut(30);
}

void loop(){
    // Writing
    // while(!sent){
    //     Wire.beginTransmission(ADDRESS);
    //     Wire.write(message);
    //     error = Wire.endTransmission();
    //     Serial.printf("[%d] ", lineNum++);
    //     checkError(error);

    //     delay(500);
    // }
    received = false;
    // READING
    while(!received){
        Wire.requestFrom(ADDRESS,size);    //strlen(message)
        if(Wire.available()) {
            int discard = 0;
            int data[size] = {0,0,0,0,0};
            for(int i = 0; i < size; i++){
                data[i] = Wire.read();
            }
            // for(int i = 0; i < size - 1; i++){
            //     Serial.printf("%x" ,data[i]);
            //     Serial.print("  ");
            // }
            int32_t combinedData = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
            Serial.print("Speed: ");
            Serial.println(combinedData);

            received = true;
            //Serial.printf("Data received: %d\n", data);
            //Serial.print("Data received: ");
            //Serial.println(data);
            Wire.flush();
            //delay(250);
        }
    }
}

void checkError(byte error){
    switch(error) {
        case 0:
            Serial.println("Transmission successful");
            sent = true;
            break;
        case 1:
            Serial.println("NACK on transmit of address");
            break;
        case 2:
            Serial.println("NACK after transmit of data byte");
            break;
        case 3:
            Serial.println("Master mode function disallowed");
            break;
        case 4:
            Serial.println("Bus error");
            break;
        case 5:
            Serial.println("Overrun error");
            break;
        default:
            Serial.println("Unknown error");
    }
}
