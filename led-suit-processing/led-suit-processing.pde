/**
 * Simple Read
 * 
 * Read data from the serial port and change the color of a rectangle
 * when a switch connected to a Wiring or Arduino board is pressed and released.
 * This example works with the Wiring / Arduino program that follows below.
 */


import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

Serial myPort;  // Create object from Serial class
int val;      // Data received from the serial port
int hash = 35;
int gt = 62;

final int bufLength = 16;

final byte[] bytes = new byte[bufLength];
final FloatBuffer buf = ByteBuffer.wrap(bytes).asFloatBuffer();
float[] floats = new float[4];
float[][] m;

quaternion q;

void setup() 
{
   noLoop();
  size(500, 500, P3D);
  // I know that the first port in the serial list on my mac
  // is always my  FTDI adaptor, so I open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String[] availablePorts = Serial.list();
  String portName = availablePorts[0];
  printArray(availablePorts);
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n');
}

void draw()
{
  background(255);
  // fill()
  translate(width/2, height/2, -100);
  applyMatrix(m[0][0], m[0][1], m[0][2], 0,
            m[1][0], m[1][1], m[1][2], 0,
            m[2][0], m[2][1], m[2][2], 0,
            0, 0, 0, 1);
  //rotateY((floats[0]));
  //rotateX((-floats[1]));
  //rotateZ((-floats[2]));
  box(300);
}

void serialEvent(Serial p){
  p.readBytes(bytes);
  //wait for three more hashes
  // for(int k = bufLength, i = 0; i < 2; i++, k--){
  //   bytes
  //   // while(p.available() <= 0){
  //   //   print("stalling");
  //   //   //stall
  //   // }
  //   print(p.read());
  // }
  for(int i = 0; i < 4; i++){
    // floats[i] = buf.get(i);
    floats[i] = get4bytesFloat(bytes, i*4);
  }
  q = new quaternion(floats[0], floats[1], floats[2], floats[3]);
  m = q.orthogonalMatrix();
  //loop();
  
  // printArray(bytes);
  // printArray(floats);
  
   redraw();
}

float get4bytesFloat(byte[] data, int offset) { 
  String hexint=hex(data[offset+3])+hex(data[offset+2])+hex(data[offset+1])+hex(data[offset]); 
  return Float.intBitsToFloat(unhex(hexint)); 
} 




/*

// Wiring / Arduino Code
// Code for sensing a switch status and writing the value to the serial port.

int switchPin = 4;                       // Switch connected to pin 4

void setup() {
  pinMode(switchPin, INPUT);             // Set pin 0 as an input
  Serial.begin(9600);                    // Start serial communication at 9600 bps
}

void loop() {
  if (digitalRead(switchPin) == HIGH) {  // If switch is ON,
    Serial.write(1);               // send 1 to Processing
  } else {                               // If the switch is not ON,
    Serial.write(0);               // send 0 to Processing
  }
  delay(100);                            // Wait 100 milliseconds
}

*/