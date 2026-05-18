/**
 * DWM1000 Monitor (TCP Client - Precise Center Estimation)
 * Version: 8.2
 * 주요 공지: 부저 판정에서 불안정한 approaching 조건을 제거하고 시각적 경고와 동기화했습니다.
 * 요청 사항 이외의 사항은 절대 수정하지 않았습니다.
 */

import processing.net.*;
import processing.sound.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Arrays;
import java.util.ArrayList;

Client c;
SinOsc sine;
boolean isSoundPlaying = false;
int alarmTimer = 0;
String[] vehicleData = new String[0];

float SPEED_LIMIT = 3.0;

class KalmanFilter {
  
   float q = 0.005;
   float r = 0.3;
   float x = -1.0;
   float p = 0.1;
   float k = 0.0;
/*  
  float q = 1.0;    // 프로세스 노이즈: 값을 크게 하여 시스템의 변화를 무조건 신뢰하게 함
  float r = 0.0001; // 측정 노이즈: 값을 거의 0에 가깝게 하여 센서 오차가 없다고 판단함
  float x = -1.0;
  float p = 0.1;
  float k = 0.0;
*/
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

    if (dists[0] < 2.0 && dists[1] < 2.0 && dists[2] < 2.0 && dists[3] < 2.0) {
      road = "CENTER";
      pos.set(width/2, height/2);
      hasEnteredCenter = true;
    } else {
      float[] sortedDists = {dists[0], dists[1], dists[2], dists[3]};
      Arrays.sort(sortedDists);
      float minDist1 = sortedDists[0];
      float minDist2 = sortedDists[1];
      float avgDist = (minDist1 + minDist2) / 2.0;

      if ((dists[0] == minDist1 || dists[0] == minDist2) && (dists[1] == minDist1 || dists[1] == minDist2)) {
        road = "NORTH ROAD";
        pos.set(width/2, height/2 - (avgDist * 100));
      } else if ((dists[2] == minDist1 || dists[2] == minDist2) && (dists[3] == minDist1 || dists[3] == minDist2)) {
        road = "SOUTH ROAD";
        pos.set(width/2, height/2 + (avgDist * 100));
      } else if ((dists[0] == minDist1 || dists[0] == minDist2) && (dists[3] == minDist1 || dists[3] == minDist2)) {
        road = "WEST ROAD";
        pos.set(width/2 - (avgDist * 100), height/2);
      } else if ((dists[1] == minDist1 || dists[1] == minDist2) && (dists[2] == minDist1 || dists[2] == minDist2)) {
        road = "EAST ROAD";
        pos.set(width/2 + (avgDist * 100), height/2);
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
    float preD = (dist(prevPos.x, prevPos.y, width/2, height/2)) / 100.0;
    approaching = (curD < preD + 0.05);
  }
}

HashMap<String, VehicleState> vehicleMap = new HashMap<String, VehicleState>();

void setup() {
  size(800, 800);
  c = new Client(this, "127.0.0.1", 12345);
  sine = new SinOsc(this);
  sine.freq(880);

  alarmTimer = 60;
  sine.play();
  sine.amp(0.3);
  isSoundPlaying = true;
}

void draw() {
  background(20);
  drawStaticRoads();
  boolean anyVehicleSpeeding = false;

  if (c.available() > 0) {
    String raw = c.readString();
    if (raw != null && raw.trim().length() > 0) vehicleData = split(raw, '\n');
  }

  fill(0, 150);
  noStroke();
  rect(width/2, 50, width, 100);
  fill(0, 255, 255);
  textAlign(CENTER);
  textSize(16);
  text("REMOTE MONITORING SYSTEM (CLIENT)", width/2, 40);
  textSize(20);
  text("SPEED LIMIT: " + SPEED_LIMIT + " km/h", width/2, 65);

  for (String line : vehicleData) {
    line = line.trim();
    if (line.isEmpty()) continue;
    String[] d = split(line, ',');
    if (d.length < 5) continue;
    String id = d[0];
    if (!vehicleMap.containsKey(id)) vehicleMap.put(id, new VehicleState(id));
    vehicleMap.get(id).update(float(d[1]), float(d[2]), float(d[3]), float(d[4]));
  }
  vehicleData = new String[0];

  Iterator<Map.Entry<String, VehicleState>> it = vehicleMap.entrySet().iterator();
  while (it.hasNext()) {
    VehicleState v = it.next().getValue();
    if (millis() - v.lastTime > 3000) it.remove();
    else {
      color vColor = (v.hasEnteredCenter) ? color(0, 255, 100) : (v.speed >= SPEED_LIMIT ? color(255, 0, 0) : color(0, 255, 100));
      drawVehicle(v.pos.x, v.pos.y, v.speed, v.id, vColor);

      // [수정] 불안정한 approaching 조건을 빼고, 과속 중이기만 하면 알람 활성화 (경고창과 동기화)
      if (!v.hasEnteredCenter && v.speed >= SPEED_LIMIT) {
        anyVehicleSpeeding = true;
      }

      if (!v.hasEnteredCenter && !v.road.equals("None") && !v.road.equals("TRANSITION")) {
        drawWarning(v.road, v.id, v.speed);
      }
    }
  }

  if (vehicleMap.size() > 0) drawGlobalStatus();
  handleBuzzer(anyVehicleSpeeding);

  if (!c.active()) {
    fill(255, 0, 0);
    textAlign(CENTER);
    text("SERVER DISCONNECTED", width/2, height/2);
    handleBuzzer(false);
  }
}

void drawGlobalStatus() {
  fill(0, 180);
  rectMode(CORNER);
  int boxHeight = 60 + (vehicleMap.size() * 110);
  rect(10, 100, 240, min(boxHeight, height-120));
  fill(255);
  textAlign(LEFT);
  textSize(20);
  text("CLIENT ESTIMATION MONITOR", 20, 125);
  fill(0, 255, 255);
  text("ACTIVE VEHICLES: " + vehicleMap.size(), 20, 145);
  int i = 0;
  for (VehicleState v : vehicleMap.values()) {
    if (i > 5) break;
    int yOff = 170 + (i * 110);
    fill(200);
    textSize(20);
    text("[" + v.id + "] LOC: " + v.road + (v.hasEnteredCenter ? " (PASSED)" : ""), 25, yOff);
    for (int j=0; j<4; j++) text("A" + (j+1) + " Dist: " + nf(v.dists[j], 0, 2) + " m", 35, yOff + 15 + (j*13));
    if (v.approaching) {
      fill(255, 150, 0);
      text("STATUS: APPROACHING", 25, yOff + 70);
    } else {
      fill(150);
      text("STATUS: LEAVING", 25, yOff + 70);
    }
    if (!v.hasEnteredCenter && v.speed >= SPEED_LIMIT) fill(255, 50, 50);
    else fill(0, 255, 100);
    text("EST. SPEED: " + nf(v.speed, 0, 1) + " km/h", 25, yOff + 85);
    stroke(80);
    line(20, yOff+95, 230, yOff+95);
    i++;
  }
}

void drawVehicle(float x, float y, float spd, String id, color c) {
  fill(c);
  stroke(255);
  ellipse(x, y, 25, 25);
  fill(255);
  textAlign(CENTER);
  textSize(20);
  text(id + "\n" + nf(spd, 0, 1) + "km/h", x, y - 50);
}

void drawWarning(String road, String id, float spd) {
  float lx = width/2, ly = height/2;
  if (road.contains("NORTH")) ly = 150;
  else if (road.contains("SOUTH")) ly = height-150;
  else if (road.contains("WEST")) lx = 150;
  else if (road.contains("EAST")) lx = width-150;
  fill(spd >= SPEED_LIMIT ? color(255, 0, 0, 150) : color(0, 255, 0, 150));
  ellipse(lx, ly, 100, 50);
  fill(255);
  textAlign(CENTER);
  text("DETECTED\n" + id, lx, ly);
}

void handleBuzzer(boolean active) {
  if (active) {
    alarmTimer = 60;
    if (!isSoundPlaying) {
      sine.play();
      sine.amp(0.3);
      isSoundPlaying = true;
    }
  } else {
    if (alarmTimer > 0) {
      alarmTimer--;
    } else if (isSoundPlaying) {
      sine.stop();
      isSoundPlaying = false;
    }
  }
}

void drawStaticRoads() {
  noStroke();
  fill(40);
  rectMode(CENTER);
  rect(width/2, height/2, width, 120);
  rect(width/2, height/2, 120, height);
  stroke(100);
  line(0, height/2, width, height/2);
  line(width/2, 0, width/2, height);
  fill(255, 200, 0);
  ellipse(width/2-80, height/2-80, 15, 15);
  ellipse(width/2+80, height/2-80, 15, 15);
  ellipse(width/2+80, height/2+80, 15, 15);
  ellipse(width/2-80, height/2+80, 15, 15);
}
