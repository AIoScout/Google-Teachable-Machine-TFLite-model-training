import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import websockets.*;
import controlP5.*;
import java.util.*;

// Main Serial port for ESP32 communication
Serial myPort;
// WebSocket server to communicate with Teachable Machine web interface
WebsocketServer ws;

// Constants - Must match ESP32 camera resolution
final int cameraWidth = 96;
final int cameraHeight = 96;
final int cameraBytesPerPixel = 1; // 1 for Grayscale
final int bytesPerFrame = cameraWidth * cameraHeight * cameraBytesPerPixel;

// Image and data buffers
PImage myImage;
byte[] frameBuffer = new byte[bytesPerFrame];
String[] portNames;
ControlP5 cp5;
ScrollableList portsList;
boolean clientConnected = false;

void setup()
{
  size(448, 224);
  pixelDensity(displayDensity());
  frameRate(30);
  
  // UI Setup
  cp5 = new ControlP5(this);
  portNames = Serial.list();
  portNames = filteredPorts(portNames); // Remove system/unnecessary ports
  
  // Start WebSocket server on port 8889
  ws = new WebsocketServer(this, 8889, "/");
  
  // Dropdown list for COM port selection
  portsList = cp5.addScrollableList("portSelect")
    .setPosition(235, 10)
    .setSize(200, 220)
    .setBarHeight(40)
    .setItemHeight(40)
    .addItems(portNames);

  portsList.close();
  
  // Initialize the image display object
  myImage = createImage(cameraWidth, cameraHeight, RGB);
  noStroke();
}

void draw()
{  
  background(240);
  // Display the captured image (scaled up for visibility)
  image(myImage, 0, 0, 224, 224);
  drawConnectionStatus();
}

// Draw a simple status indicator for Teachable Machine connection
void drawConnectionStatus() {
  fill(0);
  textAlign(RIGHT, CENTER);
  if (!clientConnected) {
    text("Not Connected to TM", 410, 100);
    fill(255, 0, 0); // Red dot for disconnected
  } else {
    text("Connected to TM", 410, 100);
    fill(0, 255, 0); // Green dot for connected
  }
  ellipse(430, 102, 10, 10);
}

// Called when a port is selected from the dropdown
void portSelect(int n) {
  String selectedPortName = (String) cp5.get(ScrollableList.class, "portSelect").getItem(n).get("text");

  try {
    // Open serial port at 921600 baud (8x faster for smooth FPS)
    myPort = new Serial(this, selectedPortName, 921600);
    println("Connected to: " + selectedPortName + " at 921600 baud");
  } 
  catch (Exception e) {
    println("Error opening serial port: " + e);
  }
}

// Filter out system ports that we don't need
boolean stringFilter(String s) {
  return (!s.startsWith("/dev/tty"));
}

String [] filteredPorts(String[] ports) {
  int n = 0;
  for (String portName : ports) if (stringFilter(portName)) n++;
  String[] retArray = new String[n];
  n = 0;
  for (String portName : ports) if (stringFilter(portName)) retArray[n++] = portName; 
  return retArray;
}

// Frame synchronization state machine variables
int syncState = 0; // 0: Searching for 0xAA, 1: Searching for 0x55, 2: Searching for 0xAA, 3: Reading pixels
int readIndex = 0;
int lastFrame = -1;

// Serial event handler - called whenever data is available on the COM port
void serialEvent(Serial myPort) {
  while (myPort.available() > 0) {
    int inByte = myPort.read();
    
    // State machine to find the sync header: 0xAA, 0x55, 0xAA
    if (syncState == 0) {
      if (inByte == 0xAA) syncState = 1;
    } 
    else if (syncState == 1) {
      if (inByte == 0x55) syncState = 2;
      else if (inByte == 0xAA) syncState = 1; // Handle consecutive 0xAA
      else syncState = 0;
    } 
    else if (syncState == 2) {
      if (inByte == 0xAA) {
        syncState = 3; // Header found! Start reading image data
        readIndex = 0;
      } else {
        syncState = 0;
      }
    } 
    else if (syncState == 3) {
      // Collect image bytes into the buffer
      frameBuffer[readIndex++] = (byte)inByte;
      
      // Once we have a full frame (9216 bytes)
      if (readIndex >= bytesPerFrame) {
        syncState = 0; // Reset state machine for next frame
        
        // Update the display image pixels
        myImage.loadPixels();
        for (int i = 0; i < bytesPerFrame; i++) {
          int r = (int) (frameBuffer[i] & 0xFF); // Grayscale value
          myImage.pixels[i] = color(r, r, r);
        }
        myImage.updatePixels();
        
        // Track time for optional FPS calculation
        lastFrame = millis();
        
        // Send the raw image bytes to Teachable Machine as a Base64 string
        String b64Data = Base64.getEncoder().encodeToString(frameBuffer);
        ws.sendMessage(b64Data);
      }
    }
  }
}

// Handle messages from the Teachable Machine web client
void webSocketServerEvent(String msg) {
  if (msg.equals("tm-connected")) clientConnected = true;
}
