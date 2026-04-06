import processing.serial.*;

Serial serialPort;
String serialPortName = "/dev/cu.usbserial-AO003MWP"; // Leave empty to use the first available port
int baudRate = 256000; // Adjust to RD-03D baud rate if different

ArrayList<Detection> detections = new ArrayList<Detection>();
ArrayList<Byte> inputBuffer = new ArrayList<Byte>();

float maxRangeMeters = 10.0;    // Adjust to the maximum radar range you want to display
float fieldOfViewDegrees = 120; // Typical mm-wave FOV; override if the RD-03D has a different FOV
float rangeUnitScale = 0.01;    // Convert raw range units to meters (default assumes cm)
float velocityUnitScale = 0.1;  // Convert raw velocity units to m/s
int maxDetections = 120;

final int PACKET_LENGTH = 30;
final byte HEADER_A = (byte)0xAA;
final byte HEADER_B = (byte)0xFF;
final byte TAIL_A = (byte)0x55;
final byte TAIL_B = (byte)0xCC;

int packetsParsed = 0;
int parseErrors = 0;

void setup() {
  size(900, 700);
  smooth();
  frameRate(60);
  textFont(createFont("Arial", 14));

  println("Available serial ports:");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println(i + ": " + ports[i]);
  }

  if (serialPortName.equals("")) {
    if (ports.length > 0) {
      serialPortName = ports[0];
      println("Using first available port: " + serialPortName);
    } else {
      println("No serial ports found. Connect the RD-03D or USB bridge and restart.");
    }
  }

  if (serialPortName.length() > 0) {
    serialPort = new Serial(this, serialPortName, baudRate);
    serialPort.clear();
  }
}

void draw() {
  background(18);
  drawCartesianBackground();
  drawDetections();
  drawHud();
}

void drawCartesianBackground() {
  pushMatrix();
  translate(width * 0.5, height * 0.55);

  float size = min(width, height) * 0.65;
  float half = size * 0.5;

  stroke(100, 180);
  strokeWeight(1);
  noFill();
  rect(-half, -half, size, size);

  int gridLines = 10;
  for (int i = 1; i < gridLines; i++) {
    float pos = lerp(-half, half, i / (float)gridLines);
    stroke(80, 140);
    line(pos, -half, pos, half);
    line(-half, pos, half, pos);
  }

  stroke(200);
  strokeWeight(2);
  line(-half, 0, half, 0);
  line(0, -half, 0, half);

  fill(200);
  noStroke();
  textAlign(RIGHT, TOP);
  text("Y", -half + 16, -half + 8);
  textAlign(LEFT, BOTTOM);
  text("X", half - 16, half - 8);

  popMatrix();
}

void drawDetections() {
  ArrayList<Detection> snapshot;
  synchronized(detections) {
    snapshot = new ArrayList<Detection>(detections);
  }

  float chartSize = min(width, height) * 0.65;
  float half = chartSize * 0.5;
  float maxRangeMm = maxRangeMeters * 1000.0;

  pushMatrix();
  translate(width * 0.5, height * 0.55);

  int[] targetColors = {color(0, 180, 255), color(0, 220, 120), color(255, 180, 40)};

  for (Detection d : snapshot) {
    if (d.valid == 0) continue;

    float px = map(d.x, -maxRangeMm, maxRangeMm, -half, half);
    float py = map(-d.y, -maxRangeMm, maxRangeMm, -half, half);

    int col = targetColors[(d.id - 1) % targetColors.length];
    stroke(col);
    fill(col, 180);
    ellipse(px, py, 1, 1);

    //fill(255);
    //textAlign(CENTER, CENTER);
    //textSize(12);
    //text("T" + d.id, px, py - 18);
  }

  popMatrix();
}

void drawHud() {
  fill(230);
  textAlign(LEFT, TOP);
  text("RD-03D Radar Display", 20, 20);
  if (serialPort != null) {
    text("Serial port: " + serialPortName + " @ " + baudRate, 20, 46);
  } else {
    text("Serial port not open. Check port selection and sensor connection.", 20, 46);
  }
  text("Max range: " + nf(maxRangeMeters, 1, 1) + " m", 20, 72);
  text("Plot mode: Cartesian XY", 20, 98);
  text("Packets: " + packetsParsed + "", 20, 124);
  text("Parse errors: " + parseErrors + "", 20, 150);

  int detectionCount;
  synchronized(detections) {
    detectionCount = detections.size();
  }

  textAlign(RIGHT, TOP);
  text("Detections: " + detectionCount, width - 20, 20);
  text("Binary packet mode", width - 20, 46);
}

void serialEvent(Serial p) {
  synchronized(inputBuffer) {
    while (p.available() > 0) {
      int nextByte = p.read();
      if (nextByte < 0) break;
      inputBuffer.add((byte) nextByte);
    }
    processInputBuffer();
  }
}

void processInputBuffer() {
  while (inputBuffer.size() >= PACKET_LENGTH) {
    if (inputBuffer.get(0) == HEADER_A && inputBuffer.get(1) == HEADER_B) {
      if (inputBuffer.get(PACKET_LENGTH - 2) == TAIL_A && inputBuffer.get(PACKET_LENGTH - 1) == TAIL_B) {
        byte[] packet = new byte[PACKET_LENGTH];
        for (int i = 0; i < PACKET_LENGTH; i++) {
          packet[i] = inputBuffer.remove(0);
        }
        parseBinaryPacket(packet);
        continue;
      } else {
        inputBuffer.remove(0);
      }
    } else {
      inputBuffer.remove(0);
    }
  }
}

void parseBinaryPacket(byte[] packet) {
  int packetType = unsigned16(packet[2], packet[3]);
  if (packetType != 0x0003) {
    parseErrors++;
    return;
  }

  synchronized(detections) {
    // Parse up to 3 targets per packet; each target block is 8 bytes:
    // X(2), Y(2), velocity(1), reserved(1), resolution(2)
    for (int targetId = 1; targetId <= 3; targetId++) {
      int offset = 4 + (targetId - 1) * 8;
      if (offset + 7 >= packet.length) break;

      int xMm = signedMag16(packet[offset], packet[offset + 1]);
      int yMm = signedMag16(packet[offset + 2], packet[offset + 3]);
      int velocityCmS = signedMag16(packet[offset + 4], packet[offset + 5]);
      int resolutionMm = unsigned16(packet[offset + 6], packet[offset + 7]);
      float velocity = velocityCmS * velocityUnitScale;

      int valid = (xMm != 0 || yMm != 0) ? 1 : 0;
      detections.add(new Detection(xMm, yMm, velocity, resolutionMm, targetId, valid));
    }
    while (detections.size() > maxDetections) {
      detections.remove(0);
    }
  }
  packetsParsed++;
}

int unsignedByte(byte b) {
  return b & 0xFF;
}

int unsigned16(byte low, byte high) {
  return unsignedByte(low) | (unsignedByte(high) << 8);
}

int signedMag16(byte low, byte high) {
  int highVal = unsignedByte(high);
  boolean positive = (highVal & 0x80) != 0;
  int magnitude = ((highVal & 0x7F) << 8) | unsignedByte(low);
  return positive ? magnitude : -magnitude;
}

class Detection {
  int x;
  int y;
  float velocity;
  int resolutionMm;
  int id;
  int valid;

  Detection(int x, int y, float velocity, int resolutionMm, int id, int valid) {
    this.x = x;
    this.y = y;
    this.velocity = velocity;
    this.resolutionMm = resolutionMm;
    this.id = id;
    this.valid = valid;
  }
}
