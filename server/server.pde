/**
 * DWM1000 Simulator (TCP Server - Raw Data & Full UI)
 * Version: 5.2
 * 주요 공지: 태그 생성 시 이미 도로에 있는 태그 번호는 생성하지 않도록 중복 방지 로직을 추가했습니다.
 * 요청 사항 이외의 사항은 절대 수정하지 않았습니다.
 */

import processing.net.*;

Server s;
PVector[] anchors = new PVector[4];
ArrayList<Vehicle> vehicles = new ArrayList<Vehicle>();

// 속도 조절 변수 (km/h 단위)
float MIN_SPEED = 1.0; 
float MAX_SPEED = 5.0;
float error_tolerance = 0.5; // 앵커 거리 오차

int lastSampleTime = 0;

int nextSampleInterval1 = 100;
int nextSampleInterval2 = 200;

int nextSampleInterval = 100;

boolean buzzerActive = false; 

void setup() {
  size(800, 800);
  s = new Server(this, 12345); 
  
  float offset = 80; 
  anchors[0] = new PVector(width/2 - offset, height/2 - offset); 
  anchors[1] = new PVector(width/2 + offset, height/2 - offset); 
  anchors[2] = new PVector(width/2 + offset, height/2 + offset); 
  anchors[3] = new PVector(width/2 - offset, height/2 + offset); 

  nextSampleInterval = (int)random(nextSampleInterval1, nextSampleInterval2);
}

void draw() {
  background(30);
  drawRoads();
  drawButtons();
  
  boolean isSampleFrame = false;
  if (millis() - lastSampleTime >= nextSampleInterval) {
    isSampleFrame = true;
    lastSampleTime = millis();
    nextSampleInterval = (int)random(nextSampleInterval1, nextSampleInterval2); 
  }

  buzzerActive = false; 

  for (int i = vehicles.size() - 1; i >= 0; i--) {
    Vehicle v = vehicles.get(i);
    v.move();
    
    // 내부 로직용 업데이트 (서버 화면 표시용)
    v.updateStatus(anchors, nextSampleInterval); 

    if (isSampleFrame) {
      // 실제 전송 데이터는 요청하신 대로 ID와 거리 4개만 포함
      float d1 = v.dists[0] + random(0, error_tolerance);
      float d2 = v.dists[1] + random(0, error_tolerance);
      float d3 = v.dists[2] + random(0, error_tolerance);
      float d4 = v.dists[3] + random(0, error_tolerance);
      
      String data = v.id + "," + nf(d1, 0, 2) + "," + nf(d2, 0, 2) + "," + 
                    nf(d3, 0, 2) + "," + nf(d4, 0, 2) + "\n";
      s.write(data); 
    }
    
    if (v.currentKmh >= 15) buzzerActive = true;
    v.display();
    if (v.isDead()) vehicles.remove(i);
  }
  
  drawAnchors();
  displayGlobalStatus(); 
  
  fill(0, 255, 255); textSize(12); textAlign(LEFT);
  text("TCP SERVER: RAW DATA MODE (UI RESTORED)", 20, height - 20);
}

void drawAnchors() {
  for (int i = 0; i < 4; i++) {
    fill(255, 200, 0); noStroke(); ellipse(anchors[i].x, anchors[i].y, 18, 18);
    fill(255); textSize(11); textAlign(CENTER); text("A" + (i+1), anchors[i].x, anchors[i].y - 15);
  }
}

void drawButtons() {
  rectMode(CENTER); textAlign(CENTER, CENTER);
  drawBtn(width/2, 30, "START N"); drawBtn(width/2, height-30, "START S");
  drawBtn(40, height/2, "START W"); drawBtn(width-40, height/2, "START E");
}

void drawBtn(float x, float y, String label) {
  fill(60); stroke(150); if (dist(mouseX, mouseY, x, y) < 40) fill(100);
  rect(x, y, 75, 35, 5); fill(255); textSize(10); text(label, x, y);
}

// 중복되지 않는 새로운 ID 생성 함수 추가
String getUniqueTagID() {
  String newId;
  boolean isDuplicate;
  do {
    isDuplicate = false;
    newId = "TAG" + (int)random(10, 19);
    for (Vehicle v : vehicles) {
      if (v.id.equals(newId)) {
        isDuplicate = true;
        break;
      }
    }
  } while (isDuplicate && vehicles.size() < 9); // 가능한 모든 ID(10~18)가 차있지 않은 동안만 반복
  return newId;
}

void mousePressed() {
  float speed = random(MIN_SPEED, MAX_SPEED); 
  String uniqueId = getUniqueTagID();
  
  if (dist(mouseX, mouseY, width/2, 30) < 40) vehicles.add(new Vehicle(width/2, -50, 0, 1, uniqueId, speed));
  else if (dist(mouseX, mouseY, width/2, height-30) < 40) vehicles.add(new Vehicle(width/2, height+50, 0, -1, uniqueId, speed));
  else if (dist(mouseX, mouseY, 40, height/2) < 40) vehicles.add(new Vehicle(-50, height/2, 1, 0, uniqueId, speed));
  else if (dist(mouseX, mouseY, width-40, height/2) < 40) vehicles.add(new Vehicle(width+50, height/2, -1, 0, uniqueId, speed));
}

void drawRoads() {
  noStroke(); fill(45); rectMode(CENTER);
  rect(width/2, height/2, width, 120); rect(width/2, height/2, 120, height);
  stroke(255, 60); line(0, height/2, width, height/2); line(width/2, 0, width/2, height);
}

void displayGlobalStatus() {
  fill(0, 200); rectMode(CORNER);
  int boxHeight = 60 + (vehicles.size() * 110);
  rect(10, 10, 240, min(boxHeight, height-20));
  fill(255); textAlign(LEFT); textSize(13);
  text("DWM1000 SYSTEM MONITOR (SRV)", 20, 35);
  if (buzzerActive) { fill(255, 0, 0); text("ALARM: ACTIVE", 180, 35); }
  else { fill(0, 255, 0); text("ALARM: CLEAR", 180, 35); }
  fill(0, 255, 255); text("ACTIVE VEHICLES: " + vehicles.size(), 20, 55);
  textSize(20);
  for (int i = 0; i < vehicles.size(); i++) {
    if (i > 5) break; 
    Vehicle v = vehicles.get(i);
    int yOff = 80 + (i * 110);
    fill(200); text("[" + v.id + "] LOC: " + v.road, 25, yOff);
    for(int j=0; j<4; j++) text("A" + (j+1) + " Dist: " + nfc(v.dists[j], 2) + " m", 35, yOff + 15 + (j*13));
    if (v.approaching) { fill(255, 150, 0); text("STATUS: APPROACHING", 25, yOff + 70); }
    else { fill(150); text("STATUS: LEAVING", 25, yOff + 70); }
    text("SPEED: " + nfc(v.currentKmh, 1) + " km/h", 25, yOff + 85);
    stroke(80); line(20, yOff+95, 230, yOff+95);
  }
}

class Vehicle {
  String id; PVector pos, prevPos, dir; float speedMps, currentKmh; float[] dists = new float[4]; String road = "None"; boolean approaching = false;
  Vehicle(float x, float y, float dx, float dy, String id, float speedKmh) { 
    this.id = id; pos = new PVector(x, y); prevPos = new PVector(x, y); dir = new PVector(dx, dy); 
    speedMps = speedKmh / 3.6; currentKmh = speedKmh; 
  }
  void move() { pos.add(PVector.mult(dir, (speedMps * 100) / 60.0)); }
  void updateStatus(PVector[] anchors, int interval) {
    float curD = dist(pos.x, pos.y, width/2, height/2);
    float preD = dist(prevPos.x, prevPos.y, width/2, height/2);
    approaching = (curD < preD - 0.01);
    for(int j=0; j<4; j++) dists[j] = dist(pos.x, pos.y, anchors[j].x, anchors[j].y) / 100.0;
    float nSum = dists[0] + dists[1]; float sSum = dists[2] + dists[3];
    float wSum = dists[0] + dists[3]; float eSum = dists[1] + dists[2];
    if (curD/100.0 < 0.6) road = "CENTER";
    else if (nSum < sSum && nSum < wSum && nSum < eSum) road = "NORTH ROAD";
    else if (sSum < nSum && sSum < wSum && sSum < eSum) road = "SOUTH ROAD";
    else if (wSum < eSum) road = "WEST ROAD";
    else road = "EAST ROAD";
    prevPos.set(pos);
  }
  void display() {
    pushMatrix(); translate(pos.x, pos.y); if (dir.x != 0) rotate(HALF_PI);
    fill(100); stroke(200); rectMode(CENTER); rect(0, 0, 20, 32); popMatrix();
    textAlign(CENTER); textSize(20); fill(255);
    text(id + ": " + nfc(currentKmh, 1) + "km/h", pos.x, pos.y - 30);
  }
  boolean isDead() { return (pos.x < -100 || pos.x > width+100 || pos.y < -100 || pos.y > height+100); }
}
