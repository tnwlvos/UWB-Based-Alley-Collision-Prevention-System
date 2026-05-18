/**
 * DWM1000 Monitor (Cyberpunk UI Version)
 * Version: 11.8
 * 주요 공지: 부저 버튼을 토글 방식에서 누르고 있는 동안만 전송되는 방식으로 변경.
 * 스타일: Processing Java 스타일 유지
 */

import processing.serial.*;
import processing.sound.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

Serial myPort;
SinOsc sine;
boolean isSoundPlaying = false;
int alarmTimer = 0;

float SPEED_LIMIT = 3.0;

/* 시리얼 연결 관련 변수 */
String[] portList;
int selectedPortIndex = -1;
boolean isConnected = false;
String connectionStatus = "OFFLINE";

/* 부저 버튼 관련 변수 */
boolean buzzerActive = false;
int lastBuzzerSendTime = 0;
int lastAutoAlarmSendTime = 0;

/* 터미널 관련 변수 - 쓰레드 안전을 위해 동기화된 리스트로 선언 */
List<String> terminalLines = Collections.synchronizedList(new ArrayList<String>());
int maxTerminalLines = 6;

class KalmanFilter {
  float q = 0.005;
  float r = 0.3;
  float x = -1.0;
  float p = 0.1;
  float k = 0.0;

  KalmanFilter(float initVal) {
    this.x = initVal;
  }

  float update(float measurement) {
    if (x == -1.0 || x == 0.0) {
      x = measurement;
      return x;
    }
    p = p + q;
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
    return x;
  }
}

class VehicleState {
  String id;
  float[] dists = new float[4];
  KalmanFilter[] kfs = new KalmanFilter[4];
  PVector pos = new PVector();
  PVector prevPos = new PVector();
  float speed = 0;
  ArrayList<Float> speedHistory = new ArrayList<Float>();
  int maxHistory = 1;
  boolean approaching = false;
  String road = "None";
  int lastTime;
  boolean hasEnteredCenter = false;
  int displayOrder = 0;

  VehicleState(String id) {
    this.id = id;
    lastTime = millis();
    for (int i = 0; i < 4; i++) {
      kfs[i] = new KalmanFilter(-1.0);
    }
  }

  void update(float d1, float d2, float d3, float d4) {
    int now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    dists[0] = kfs[0].update(d1);
    dists[1] = kfs[1].update(d2);
    dists[2] = kfs[2].update(d3);
    dists[3] = kfs[3].update(d4);

    prevPos.set(pos.x, pos.y);

    float centerX = 400;
    float centerY = 400;

    if (dists[0] < 2.0 && dists[1] < 2.0 && dists[2] < 2.0 && dists[3] < 2.0) {
      road = "CENTER";
      pos.set(centerX, centerY);
      hasEnteredCenter = true;
    } else {
      float[] sortedDists = {dists[0], dists[1], dists[2], dists[3]};
      Arrays.sort(sortedDists);
      float minDist1 = sortedDists[0];
      float minDist2 = sortedDists[1];
      float avgDist = (minDist1 + minDist2) / 2.0;

      if ((dists[0] == minDist1 || dists[0] == minDist2) && (dists[1] == minDist1 || dists[1] == minDist2)) {
        road = "NORTH ROAD";
        pos.set(centerX, centerY - (avgDist * 100));
      } else if ((dists[2] == minDist1 || dists[2] == minDist2) && (dists[3] == minDist1 || dists[3] == minDist2)) {
        road = "SOUTH ROAD";
        pos.set(centerX, centerY + (avgDist * 100));
      } else if ((dists[0] == minDist1 || dists[0] == minDist2) && (dists[3] == minDist1 || dists[3] == minDist2)) {
        road = "WEST ROAD";
        pos.set(centerX - (avgDist * 100), centerY);
      } else if ((dists[1] == minDist1 || dists[1] == minDist2) && (dists[2] == minDist1 || dists[2] == minDist2)) {
        road = "EAST ROAD";
        pos.set(centerX + (avgDist * 100), centerY);
      } else {
        road = "TRANSITION";
      }
    }

    if (dt > 0) {
      float distanceMoved = dist(pos.x, pos.y, prevPos.x, prevPos.y) / 100.0;
      float instantSpeed = (distanceMoved / dt) * 3.6;
      speedHistory.add(instantSpeed);
      if (speedHistory.size() > maxHistory) speedHistory.remove(0);
      float sum = 0;
      for (float s : speedHistory) sum += s;
      speed = sum / speedHistory.size();
    }

    float curD = (dists[0] + dists[1] + dists[2] + dists[3]) / 4.0;
    float preD = (dist(prevPos.x, prevPos.y, centerX, centerY)) / 100.0;
    approaching = (curD < preD + 0.05);
  }
}

HashMap<String, VehicleState> vehicleMap = new HashMap<String, VehicleState>();

void setup() {
  size(1000, 950);
  portList = Serial.list();

  sine = new SinOsc(this);
  sine.freq(880);
}

void draw() {
  background(10, 12, 18);
  drawGrid();
  drawDashboard();

  pushMatrix();
  translate(200, 0);
  drawStaticRoads();

  boolean anyVehicleSpeeding = false;

  int idx = 0;
  Iterator<Map.Entry<String, VehicleState>> it = vehicleMap.entrySet().iterator();
  while (it.hasNext()) {
    VehicleState v = it.next().getValue();
    if (millis() - v.lastTime > 3000) it.remove();
    else {
      v.displayOrder = idx;
      color vColor = (v.hasEnteredCenter) ? color(0, 255, 180) : (v.speed >= SPEED_LIMIT ? color(255, 45, 85) : color(0, 240, 255));
      drawVehicle(v.pos.x, v.pos.y, v.speed, v.id, vColor);

      if (!v.hasEnteredCenter && v.speed >= SPEED_LIMIT) anyVehicleSpeeding = true;
      if (!v.hasEnteredCenter && !v.road.equals("None") && !v.road.equals("TRANSITION")) {
        drawWarning(v.road, v.id, v.speed, v.displayOrder);
      }
      idx++;
    }
  }

  if (vehicleMap.size() > 0) drawGlobalStatus();
  handleBuzzer(anyVehicleSpeeding);
  popMatrix();

  drawTerminal();

  if (isConnected) {
    // anyVehicleSpeeding: draw() 루프 상단에서 과속 차량이 한 대라도 있으면 true가 됨
    // buzzerActive: 사용자가 버튼을 누르고 있는 경우
    if (anyVehicleSpeeding || buzzerActive) {
      if (millis() - lastAutoAlarmSendTime >= 100) {
        myPort.write('1');
        lastAutoAlarmSendTime = millis();
      }
    }
  }

  if (isConnected && buzzerActive) {
    if (millis() - lastBuzzerSendTime >= 100) {
      myPort.write('1');
      lastBuzzerSendTime = millis();
    }
  }
}

void drawGrid() {
  stroke(30, 40, 60);
  strokeWeight(1);
  for (int i=0; i<width; i+=50) line(i, 0, i, height);
  for (int j=0; j<height; j+=50) line(0, j, width, j);
}

void drawDashboard() {
  fill(15, 18, 25, 220);
  noStroke();
  rect(0, 0, 200, height);
  stroke(0, 255, 255, 100);
  line(200, 0, 200, height);

  fill(0, 255, 255);
  textAlign(LEFT, TOP);
  textSize(18);
  text("CONSOLE", 15, 40);

  for (int i = 0; i < min(portList.length, 15); i++) {
    rectMode(CORNER);
    if (i == selectedPortIndex) {
      fill(0, 255, 255, 50);
      stroke(0, 255, 255);
    } else {
      fill(30, 35, 45);
      noStroke();
    }
    rect(10, 60 + (i * 35), 180, 28, 4);

    fill(255);
    textAlign(LEFT, TOP);
    textSize(11);
    text(portList[i], 22, 68 + (i * 35));
  }

  float bX = 15;
  float bY = height - 165;
  float bW = 170;
  float bH = 45;

  noStroke();
  rectMode(CORNER);
  if (isConnected) fill(255, 45, 85);
  else fill(0, 240, 255);
  rect(bX, bY, bW, bH, 8);

  textAlign(CENTER, CENTER);
  textSize(14);
  fill(isConnected ? 255 : 0);
  text(isConnected ? "DISCONNECT" : "CONNECT", bX + (bW/2.0), bY + (bH/2.0));

  float buzzY = height - 105;
  if (buzzerActive) fill(255, 200, 0);
  else fill(45, 50, 65);
  rect(bX, buzzY, bW, bH, 8);

  fill(buzzerActive ? 0 : 255);
  text("BUZZER", bX + (bW/2.0), buzzY + (bH/2.0));

  fill(isConnected ? color(0, 255, 180) : color(255, 100, 100));
  textSize(11);
  textAlign(CENTER, CENTER);
  text("SYS.STATUS: " + connectionStatus, 100, bY - 20);
}

void drawTerminal()
{
  fill(10, 15, 20, 230);
  stroke(0, 255, 255, 50);
  rectMode(CORNER);
  rect(210, 820-20, width - 220, 170-50, 5);

  fill(0, 255, 255, 150);
  textSize(10);
  textAlign(LEFT, TOP);
  text("SERIAL MONITOR OUTPUT", 220, 827-20);

  fill(0, 255, 180);
  textSize(12);

  synchronized(terminalLines) {
    int lineCount = 0;
    for (String line : terminalLines) {
      if (lineCount < maxTerminalLines) {
        text("> " + line, 220, 845 + (lineCount * 16)-20);
        lineCount++;
      }
    }
  }
}

void mousePressed() {
  for (int i = 0; i < min(portList.length, 15); i++) {
    if (mouseX > 10 && mouseX < 190 && mouseY > 60 + (i * 35) && mouseY < 88 + (i * 35)) {
      selectedPortIndex = i;
    }
  }

  if (mouseX >= 15 && mouseX <= 185 && mouseY >= height - 165 && mouseY <= height - 120) {
    if (!isConnected && selectedPortIndex != -1) {
      try {
        myPort = new Serial(this, portList[selectedPortIndex], 115200);
        myPort.bufferUntil('\n');
        isConnected = true;
        connectionStatus = "ONLINE";
        synchronized(terminalLines) {
          terminalLines.add("Connected to " + portList[selectedPortIndex]);
        }
      }
      catch (Exception e) {
        connectionStatus = "ERR: BUSY";
      }
    } else if (isConnected) {
      myPort.stop();
      isConnected = false;
      buzzerActive = false;
      connectionStatus = "OFFLINE";
      synchronized(terminalLines) {
        terminalLines.add("Disconnected.");
        vehicleMap.clear();
      }
    }
  }

  // [수정] 누르고 있는 동안만 활성화
  if (mouseX >= 15 && mouseX <= 185 && mouseY >= height - 105 && mouseY <= height - 60) {
    if (isConnected) {
      buzzerActive = true;
    }
  }
}

// [추가] 마우스를 떼면 부저 비활성화
void mouseReleased() {
  buzzerActive = false;
}

void serialEvent(Serial p) {
  try {
    String inString = p.readString();
    if (inString != null) {
      inString = trim(inString);

      synchronized(terminalLines) {
        terminalLines.add(inString);
        if (terminalLines.size() > maxTerminalLines) terminalLines.remove(0);
      }

      String[] parts = split(inString, ':');
      if (parts.length == 2) {
        String id = parts[0];
        String[] distStrs = split(parts[1], ',');
        if (distStrs.length == 4) {
          if (!vehicleMap.containsKey(id)) vehicleMap.put(id, new VehicleState(id));
          vehicleMap.get(id).update(
            float(distStrs[0])/1000.0,
            float(distStrs[1])/1000.0,
            float(distStrs[2])/1000.0,
            float(distStrs[3])/1000.0
            );
        }
      }
    }
  }
  catch (Exception e) {
  }
}

void drawGlobalStatus() {
  float baseX = 10;
  float baseY = 10;
  float panelW = 240;
  float itemH = 115;

  fill(15, 18, 25, 200);
  stroke(0, 255, 255, 80);
  rectMode(CORNER);
  rect(baseX, baseY, panelW, 60 + (vehicleMap.size() * itemH), 10);

  fill(0, 255, 255);
  textAlign(LEFT, TOP);
  textSize(16);
  text("RADAR FEED", baseX + 15, baseY + 25);

  int i = 0;
  for (VehicleState v : vehicleMap.values()) {
    if (i > 5) break;
    float yOff = baseY + 55 + (i * itemH);

    fill(255, 20);
    noStroke();
    rect(baseX + 10, yOff - 20, panelW - 20, 105, 5);

    fill(255);
    textSize(14);
    text("ID: " + v.id, baseX + 20, yOff);
    fill(200);
    textSize(11);
    text("LOC: " + v.road, baseX + 110, yOff);

    for (int j = 0; j < 4; j++) {
      fill(0, 255, 255, 150);
      text("A" + j + ": " + nf(v.dists[j], 0, 2) + "m", baseX + 25 + (j % 2) * 90, yOff + 20 + (j / 2) * 18);
    }

    if (!v.hasEnteredCenter && v.speed >= SPEED_LIMIT) fill(255, 45, 85);
    else fill(0, 255, 180);
    textSize(13);
    text("SPEED: " + nf(v.speed, 0, 1) + " km/h", baseX + 20, yOff + 55);

    i++;
  }
}

void drawVehicle(float x, float y, float spd, String id, color c) {
  noStroke();
  for (int i=1; i<5; i++) {
    fill(c, 50/i);
    ellipse(x, y, 25 + i*8, 25 + i*8);
  }
  fill(c);
  ellipse(x, y, 25, 25);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(14);
  text(id, x, y - 45);
  textSize(16);
  text(nf(spd, 0, 1) + " km/h", x, y - 25);
}

void drawWarning(String road, String id, float spd, int order)
{
  float lx = 400 - 200, ly = 400;
  if (road.contains("NORTH")) ly = 180;
  else if (road.contains("SOUTH")) ly = 800 - 180;
  else if (road.contains("WEST")) lx = 180 - 200;
  else if (road.contains("EAST")) lx = 800 - 180 - 200;

  ly += (order * 65);

  fill(spd >= SPEED_LIMIT ? color(255, 45, 85, 80) : color(0, 255, 180, 80));
  stroke(255, 150);
  rectMode(CENTER);
  rect(lx, ly, 120, 60, 10);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(12);
  text("SCANNING\n" + id, lx, ly);
  rectMode(CORNER);
}

void handleBuzzer(boolean active) {
  if (active) {
    alarmTimer = 60;
    if (!isSoundPlaying) {
      sine.play();
      sine.amp(0.2);
      isSoundPlaying = true;
    }
  } else {
    if (alarmTimer > 0) alarmTimer--;
    else if (isSoundPlaying) {
      sine.stop();
      isSoundPlaying = false;
    }
  }
}
void drawStaticRoads() {
  noStroke();
  fill(25, 30, 45);
  rectMode(CENTER);

  // 실제 앵커 간 거리 기준
  // A0-A1 = 919cm = 9.19m
  // A0-A3 = 1740cm = 17.40m
  // 화면 스케일: 1m = 40px
  float SCALE = 40.0;

  float centerX = 400;
  float centerY = 400;

  float mapW = 9.19 * SCALE;   // 약 367.6px
  float mapH = 17.40 * SCALE;  // 약 696px

  // 도로 두께
  float roadThickness = 140;

  // 가로 도로, 세로 도로
  rect(centerX, centerY, mapW + 200, roadThickness);
  rect(centerX, centerY, roadThickness, mapH + 100);

  stroke(50, 70, 100);
  strokeWeight(2);

  // 도로 중앙선
  line(centerX - (mapW + 200)/2, centerY, centerX + (mapW + 200)/2, centerY);
  line(centerX, centerY - (mapH + 100)/2, centerX, centerY + (mapH + 100)/2);

  textAlign(CENTER, CENTER);
  textSize(12);

  // 앵커 좌표
  float[][] anchors = {
    {centerX - mapW/2, centerY - mapH/2}, // A0
    {centerX + mapW/2, centerY - mapH/2}, // A1
    {centerX + mapW/2, centerY + mapH/2}, // A2
    {centerX - mapW/2, centerY + mapH/2}  // A3
  };

  for (int i = 0; i < 4; i++) {
    noStroke();
    fill(255, 200, 0, 50);
    ellipse(anchors[i][0], anchors[i][1], 35, 35);

    fill(255, 200, 0);
    ellipse(anchors[i][0], anchors[i][1], 24, 24);

    fill(0);
    text(i, anchors[i][0], anchors[i][1]);
  }

  rectMode(CORNER);
}
//void drawStaticRoads() {
//  noStroke();
//  fill(25, 30, 45);
//  rectMode(CENTER);
//  rect(400, 400, 800, 140);
//  rect(400, 400, 140, 800);

//  stroke(50, 70, 100);
//  strokeWeight(2);
//  line(0, 400, 800, 400);
//  line(400, 0, 400, 800);

//  textAlign(CENTER, CENTER);
//  textSize(12);
//  float[][] anchors = {{400-80, 400-80}, {400+80, 400-80}, {400+80, 400+80}, {400-80, 400+80}};
//  for (int i=0; i<4; i++) {
//    noStroke();
//    fill(255, 200, 0, 50);
//    ellipse(anchors[i][0], anchors[i][1], 35, 35);
//    fill(255, 200, 0);
//    ellipse(anchors[i][0], anchors[i][1], 24, 24);
//    fill(0);
//    text(i, anchors[i][0], anchors[i][1]);
//  }
//}
