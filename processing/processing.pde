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

final float MAP_SCALE = 14.0;        // pixels per meter for the demo map
final float A0_A1_WIDTH_M = 6.85;    // A0-A1 road width
final float A0_A3_WIDTH_M = 7.73;    // A0-A3 road width
final float A0_A1_ROAD_LEN_M = 15.1; // road length from A0-A1 anchor line
final float A0_A3_ROAD_LEN_M = 38.0; // road length from A0-A3 anchor line

final float A0_X = 620;
final float A0_Y = 360;
final float A1_X = A0_X + A0_A1_WIDTH_M * MAP_SCALE;
final float A1_Y = A0_Y;
final float A3_X = A0_X;
final float A3_Y = A0_Y + A0_A3_WIDTH_M * MAP_SCALE;
final float A2_X = A1_X;
final float A2_Y = A3_Y;

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
    if (measurement <= 0.0) {
      return max(x, 0.0);
    }
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
  String lastRoad = "A0-A3 ROAD";
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

    float centerX = A0_X;
    float centerY = A0_Y;

    boolean hasD0 = d1 > 0.0;
    boolean hasD1 = d2 > 0.0;
    boolean hasD2 = d3 > 0.0;
    boolean hasD3 = d4 > 0.0;

    if (hasD0 && dists[0] < 2.0) {
      road = "CENTER";
      pos.set(centerX, centerY);
      hasEnteredCenter = true;
    } else {
      if (hasD0 && hasD3) {
        road = "A0-A3 ROAD";
        pos.set(projectA0A3Road(dists[0], dists[3]));
      } else if (hasD0 && hasD1) {
        road = "A0-A1 ROAD";
        pos.set(projectA0A1Road(dists[0], dists[1]));
      } else if (hasD2 && hasD3) {
        float avgDist = (dists[2] + dists[3]) / 2.0;
        road = "A2-A3 ROAD";
        pos.set(A0_X - constrain(avgDist, 0, A0_A3_ROAD_LEN_M) * MAP_SCALE, A3_Y);
      } else if (hasD1 && hasD2) {
        float avgDist = (dists[1] + dists[2]) / 2.0;
        road = "A1-A2 ROAD";
        pos.set(A1_X, A0_Y - constrain(avgDist, 0, A0_A1_ROAD_LEN_M) * MAP_SCALE);
      } else if (hasD0) {
        road = lastRoad;
        pos.set(projectSingleAnchorRoad(0, dists[0], road));
      } else if (hasD3) {
        road = "A0-A3 ROAD";
        pos.set(projectSingleAnchorRoad(3, dists[3], road));
      } else if (hasD1) {
        road = "A0-A1 ROAD";
        pos.set(projectSingleAnchorRoad(1, dists[1], road));
      } else if (hasD2) {
        road = lastRoad.equals("A0-A1 ROAD") ? "A0-A1 ROAD" : "A0-A3 ROAD";
        pos.set(projectSingleAnchorRoad(2, dists[2], road));
      } else {
        road = "PARTIAL";
      }
    }

    if (!road.equals("PARTIAL") && !road.equals("CENTER")) {
      lastRoad = road;
    }

    if (dt > 0) {
      float distanceMoved = dist(pos.x, pos.y, prevPos.x, prevPos.y) / MAP_SCALE;
      float instantSpeed = (distanceMoved / dt) * 3.6;
      speedHistory.add(instantSpeed);
      if (speedHistory.size() > maxHistory) speedHistory.remove(0);
      float sum = 0;
      for (float s : speedHistory) sum += s;
      speed = sum / speedHistory.size();
    }

    float curD = dist(pos.x, pos.y, centerX, centerY) / MAP_SCALE;
    float preD = (dist(prevPos.x, prevPos.y, centerX, centerY)) / MAP_SCALE;
    approaching = (curD < preD + 0.05);
  }
}

PVector projectA0A3Road(float dA0, float dA3) {
  float lateralFromA0 = (sq(dA0) - sq(dA3) + sq(A0_A3_WIDTH_M)) / (2.0 * A0_A3_WIDTH_M);
  lateralFromA0 = constrain(lateralFromA0, 0, A0_A3_WIDTH_M);

  float along = sqrt(max(0, sq(dA0) - sq(lateralFromA0)));
  along = constrain(along, 0, A0_A3_ROAD_LEN_M);

  return new PVector(A0_X - along * MAP_SCALE, A0_Y + lateralFromA0 * MAP_SCALE);
}

PVector projectA0A1Road(float dA0, float dA1) {
  float lateralFromA0 = (sq(dA0) - sq(dA1) + sq(A0_A1_WIDTH_M)) / (2.0 * A0_A1_WIDTH_M);
  lateralFromA0 = constrain(lateralFromA0, 0, A0_A1_WIDTH_M);

  float along = sqrt(max(0, sq(dA0) - sq(lateralFromA0)));
  along = constrain(along, 0, A0_A1_ROAD_LEN_M);

  return new PVector(A0_X + lateralFromA0 * MAP_SCALE, A0_Y - along * MAP_SCALE);
}

PVector projectSingleAnchorRoad(int anchorId, float distanceM, String road) {
  float along = max(0, distanceM);

  if (road.equals("A0-A1 ROAD")) {
    if (anchorId == 2) {
      return new PVector(A1_X, A2_Y - constrain(along, 0, A0_A1_ROAD_LEN_M) * MAP_SCALE);
    }
    if (anchorId == 1) {
      return new PVector(A1_X, A1_Y - constrain(along, 0, A0_A1_ROAD_LEN_M) * MAP_SCALE);
    }
    return new PVector(A0_X, A0_Y - constrain(along, 0, A0_A1_ROAD_LEN_M) * MAP_SCALE);
  }

  if (anchorId == 2) {
    return new PVector(A2_X - constrain(along, 0, A0_A3_ROAD_LEN_M) * MAP_SCALE, A3_Y);
  }
  if (anchorId == 3) {
    return new PVector(A0_X - constrain(along, 0, A0_A3_ROAD_LEN_M) * MAP_SCALE, A3_Y);
  }
  return new PVector(A0_X - constrain(along, 0, A0_A3_ROAD_LEN_M) * MAP_SCALE, A0_Y);
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
  float lx = A0_X - 200;
  float ly = A0_Y;
  if (road.contains("A0-A1") || road.contains("A1-A2")) ly = A0_Y - 180;
  else if (road.contains("A0-A3") || road.contains("A2-A3")) lx = A0_X - 420 - 200;

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
  rectMode(CORNER);

  fill(25, 30, 45, 230);
  float a0a3RoadX = A0_X - A0_A3_ROAD_LEN_M * MAP_SCALE;
  float a0a3RoadY = A0_Y;
  float a0a3RoadW = A0_A3_ROAD_LEN_M * MAP_SCALE;
  float a0a3RoadH = A0_A3_WIDTH_M * MAP_SCALE;
  rect(a0a3RoadX, a0a3RoadY, a0a3RoadW, a0a3RoadH, 8);

  float a0a1RoadX = A0_X;
  float a0a1RoadY = A0_Y - A0_A1_ROAD_LEN_M * MAP_SCALE;
  float a0a1RoadW = A0_A1_WIDTH_M * MAP_SCALE;
  float a0a1RoadH = A0_A1_ROAD_LEN_M * MAP_SCALE;
  rect(a0a1RoadX, a0a1RoadY, a0a1RoadW, a0a1RoadH, 8);

  stroke(50, 70, 100);
  strokeWeight(2);
  line(a0a3RoadX, A0_Y + a0a3RoadH / 2, A0_X, A0_Y + a0a3RoadH / 2);
  line(A0_X + a0a1RoadW / 2, a0a1RoadY, A0_X + a0a1RoadW / 2, A0_Y);
  drawDistanceScale();

  drawAnchor(0, A0_X, A0_Y);
  drawAnchor(1, A1_X, A1_Y);
  drawAnchor(2, A2_X, A2_Y);
  drawAnchor(3, A3_X, A3_Y);

  fill(180, 210, 255);
  textAlign(LEFT, TOP);
  textSize(12);
  text("A0-A3 width: " + nf(A0_A3_WIDTH_M, 0, 2) + "m / road: " + nf(A0_A3_ROAD_LEN_M, 0, 1) + "m", 20, 815);
  text("A0-A1 width: " + nf(A0_A1_WIDTH_M, 0, 2) + "m / road: " + nf(A0_A1_ROAD_LEN_M, 0, 1) + "m", 20, 832);
}

void drawDistanceScale() {
  stroke(120, 170, 220, 170);
  strokeWeight(1);
  fill(200, 225, 255);
  textSize(10);
  textAlign(CENTER, CENTER);

  float hCenterY = A0_Y + (A0_A3_WIDTH_M * MAP_SCALE) / 2.0;
  for (float m = 0; m <= A0_A3_ROAD_LEN_M; m += 5.0) {
    float x = A0_X - m * MAP_SCALE;
    line(x, hCenterY - 7, x, hCenterY + 7);
    text(nf(m, 0, 0) + "m", x, hCenterY + 20);
  }
  float xEnd = A0_X - A0_A3_ROAD_LEN_M * MAP_SCALE;
  line(xEnd, hCenterY - 10, xEnd, hCenterY + 10);
  text(nf(A0_A3_ROAD_LEN_M, 0, 1) + "m", xEnd, hCenterY - 22);

  float vCenterX = A0_X + (A0_A1_WIDTH_M * MAP_SCALE) / 2.0;
  for (float m = 0; m <= A0_A1_ROAD_LEN_M; m += 5.0) {
    float y = A0_Y - m * MAP_SCALE;
    line(vCenterX - 7, y, vCenterX + 7, y);
    text(nf(m, 0, 0) + "m", vCenterX + 28, y);
  }
  float yEnd = A0_Y - A0_A1_ROAD_LEN_M * MAP_SCALE;
  line(vCenterX - 10, yEnd, vCenterX + 10, yEnd);
  text(nf(A0_A1_ROAD_LEN_M, 0, 1) + "m", vCenterX - 34, yEnd);

  textAlign(LEFT, TOP);
  fill(170, 205, 255);
  text("origin A0 (0m)", A0_X + 8, A0_Y + 8);
}

void drawAnchor(int id, float x, float y) {
  noStroke();
  fill(255, 200, 0, 55);
  ellipse(x, y, 35, 35);
  fill(255, 200, 0);
  ellipse(x, y, 24, 24);
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(12);
  text(id, x, y);

  fill(255, 230, 120);
  textSize(10);
  String label = "";
  if (id == 0) label = "A0 (0,0)";
  if (id == 1) label = "A1 +" + nf(A0_A1_WIDTH_M, 0, 2) + "m";
  if (id == 2) label = "A2 corner";
  if (id == 3) label = "A3 +" + nf(A0_A3_WIDTH_M, 0, 2) + "m";
  text(label, x, y + 25);
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
