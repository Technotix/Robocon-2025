#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char* ssid = "OPPO Reno6";
const char* password = "technotix";

WebServer server(80);
WebSocketsServer ws(81);

String frontLeft = "0", frontRight = "0", rearLeft = "0", rearRight = "0";
String encoder1 = "0", encoder2 = "0", encoder3 = "0";
String robotX = "0", robotY = "0", robotRotation = "0";
String ballState = "0", flapsOpen = "0";
String isOdriveShooting = "0";
String odriveVbusVoltage = "0", odriveVelocity0 = "0", odriveVelocity1 = "0";
String odriveCurrent0 = "0", odriveCurrent1 = "0";
String odriveError0 = "0", odriveError1 = "0";
String odriveAxisError0 = "0", odriveAxisError1 = "0";
String odriveEncoderError0 = "0", odriveEncoderError1 = "0";
String odriveControllerError0 = "0", odriveControllerError1 = "0";
unsigned long lastUpdate = 0;

String getPage() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'><title>Robot Control</title><style>";
  h += "body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);margin:0;padding:20px;min-height:100vh}";
  h += ".container{max-width:900px;margin:0 auto;background:rgba(255,255,255,0.95);border-radius:15px;padding:30px;box-shadow:0 8px 32px rgba(0,0,0,0.2);backdrop-filter:blur(10px)}";
  h += "h1{text-align:center;color:#2c3e50;margin-bottom:30px;font-size:28px;text-shadow:0 2px 4px rgba(0,0,0,0.1)}";
  h += ".section{margin-bottom:25px;padding:20px;background:rgba(248,249,250,0.8);border-radius:12px;border-left:4px solid #3498db}";
  h += ".section h2{margin:0 0 20px 0;color:#34495e;font-size:20px;font-weight:600}";
  h += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:15px}";
  h += ".item{text-align:center;padding:15px;background:#fff;border-radius:10px;border:1px solid #e9ecef;transition:transform 0.2s,box-shadow 0.2s}";
  h += ".item:hover{transform:translateY(-2px);box-shadow:0 4px 12px rgba(0,0,0,0.15)}";
  h += ".label{font-size:13px;color:#6c757d;margin-bottom:8px;font-weight:500;text-transform:uppercase;letter-spacing:0.5px}";
  h += ".value{font-size:20px;font-weight:bold;margin-bottom:5px}";
  h += ".unit{font-size:12px;color:#6c757d;font-weight:normal}";
  h += ".status-section{background:rgba(248,249,250,0.8);border-left:4px solid #e74c3c}";
  h += ".shooting-active{background:#d4edda;border:2px solid #28a745;color:#155724}";
  h += ".shooting-inactive{background:#f8d7da;border:2px solid #dc3545;color:#721c24}";
  h += ".ball-idle{background:#f8f9fa;border:2px solid #6c757d;color:#495057}";
  h += ".ball-shot{background:#fff3cd;border:2px solid #ffc107;color:#856404}";
  h += ".ball-waiting{background:#cce5ff;border:2px solid #007bff;color:#004085}";
  h += ".status{text-align:center;color:#6c757d;font-size:14px;margin-top:20px;padding:10px;background:#f8f9fa;border-radius:6px}";
  h += ".error-ok{color:#28a745}.error-warning{color:#ffc107}.error-critical{color:#dc3545}";
  h += "</style></head><body>";
  h += "<div class='container'><h1>Robot Dashboard</h1>";
  h += "<div class='section'><h2>Motors</h2><div class='grid'>";
  h += "<div class='item'><div class='label'>Front Left</div><div class='value' id='fl'>0</div></div>";
  h += "<div class='item'><div class='label'>Front Right</div><div class='value' id='fr'>0</div></div>";
  h += "<div class='item'><div class='label'>Rear Left</div><div class='value' id='rl'>0</div></div>";
  h += "<div class='item'><div class='label'>Rear Right</div><div class='value' id='rr'>0</div></div>";
  h += "</div></div>";
  h += "<div class='section'><h2>Encoders</h2><div class='grid'>";
  h += "<div class='item'><div class='label'>Encoder 1</div><div class='value' id='e1'>0</div><div class='unit'>cm</div></div>";
  h += "<div class='item'><div class='label'>Encoder 2</div><div class='value' id='e2'>0</div><div class='unit'>cm</div></div>";
  h += "<div class='item'><div class='label'>Encoder 3</div><div class='value' id='e3'>0</div><div class='unit'>cm</div></div>";
  h += "</div></div>";
  h += "<div class='section'><h2>ODrive Diagnostics</h2><div class='grid'>";
  h += "<div class='item'><div class='label'>Vbus Voltage</div><div class='value' id='vbus'>0</div><div class='unit'>V</div></div>";
  h += "<div class='item'><div class='label'>Motor 0 Velocity</div><div class='value' id='vel0'>0</div><div class='unit'>rev/s</div></div>";
  h += "<div class='item'><div class='label'>Motor 1 Velocity</div><div class='value' id='vel1'>0</div><div class='unit'>rev/s</div></div>";
  h += "<div class='item'><div class='label'>Motor 0 Current</div><div class='value' id='cur0'>0</div><div class='unit'>A</div></div>";
  h += "<div class='item'><div class='label'>Motor 1 Current</div><div class='value' id='cur1'>0</div><div class='unit'>A</div></div>";
  h += "</div></div>";
  h += "<div class='section'><h2>ODrive Errors</h2><div class='grid'>";
  h += "<div class='item'><div class='label'>Motor 0 Axis Error</div><div class='value' id='axerr0'>0</div></div>";
  h += "<div class='item'><div class='label'>Motor 1 Axis Error</div><div class='value' id='axerr1'>0</div></div>";
  h += "<div class='item'><div class='label'>Motor 0 Encoder Error</div><div class='value' id='encerr0'>0</div></div>";
  h += "<div class='item'><div class='label'>Motor 1 Encoder Error</div><div class='value' id='encerr1'>0</div></div>";
  h += "<div class='item'><div class='label'>Motor 0 Controller Error</div><div class='value' id='conerr0'>0</div></div>";
  h += "<div class='item'><div class='label'>Motor 1 Controller Error</div><div class='value' id='conerr1'>0</div></div>";
  h += "</div></div>";
  h += "<div class='section'><h2>System Status</h2><div class='grid'>";
  h += "<div class='item' id='shoot-status'><div class='label'>ODrive Shooting</div><div class='value' id='shooting'>INACTIVE</div></div>";
  h += "<div class='item' id='ball-status'><div class='label'>Ball State</div><div class='value' id='ball'>0</div></div>";
  h += "<div class='item' id='flaps-status'><div class='label'>Flaps</div><div class='value' id='flaps'>CLOSED</div></div>";
  h += "</div>";
  h += "</div>";
  h += "<div class='status'>Last Update: <span id='lu'>No data</span></div></div>";
  h += "<script>";
  h += "function getMotorColor(speed){const s=Math.abs(parseInt(speed));if(s===0)return '#6c757d';if(s<=25)return '#28a745';if(s<=50)return '#ffc107';if(s<=75)return '#fd7e14';return '#dc3545'}";
  h += "let w;function c(){w=new WebSocket('ws://'+location.hostname+':81');";
  h += "w.onmessage=e=>{const d=JSON.parse(e.data);";
  h += "const fl=document.getElementById('fl');fl.innerText=d.frontLeft;fl.style.color=getMotorColor(d.frontLeft);";
  h += "const fr=document.getElementById('fr');fr.innerText=d.frontRight;fr.style.color=getMotorColor(d.frontRight);";
  h += "const rl=document.getElementById('rl');rl.innerText=d.rearLeft;rl.style.color=getMotorColor(d.rearLeft);";
  h += "const rr=document.getElementById('rr');rr.innerText=d.rearRight;rr.style.color=getMotorColor(d.rearRight);";
  h += "document.getElementById('e1').innerText=d.encoder1;";
  h += "document.getElementById('e2').innerText=d.encoder2;";
  h += "document.getElementById('e3').innerText=d.encoder3;";
  h += "document.getElementById('vbus').innerText=d.odriveVbusVoltage;";
  h += "document.getElementById('vel0').innerText=d.odriveVelocity0;";
  h += "document.getElementById('vel1').innerText=d.odriveVelocity1;";
  h += "document.getElementById('cur0').innerText=d.odriveCurrent0;";
  h += "document.getElementById('cur1').innerText=d.odriveCurrent1;";
  h += "const axerr0El=document.getElementById('axerr0');axerr0El.innerText=d.odriveAxisError0;axerr0El.className=d.odriveAxisError0==='0'?'value error-ok':'value error-critical';";
  h += "const axerr1El=document.getElementById('axerr1');axerr1El.innerText=d.odriveAxisError1;axerr1El.className=d.odriveAxisError1==='0'?'value error-ok':'value error-critical';";
  h += "const encerr0El=document.getElementById('encerr0');encerr0El.innerText=d.odriveEncoderError0;encerr0El.className=d.odriveEncoderError0==='0'?'value error-ok':'value error-critical';";
  h += "const encerr1El=document.getElementById('encerr1');encerr1El.innerText=d.odriveEncoderError1;encerr1El.className=d.odriveEncoderError1==='0'?'value error-ok':'value error-critical';";
  h += "const conerr0El=document.getElementById('conerr0');conerr0El.innerText=d.odriveControllerError0;conerr0El.className=d.odriveControllerError0==='0'?'value error-ok':'value error-critical';";
  h += "const conerr1El=document.getElementById('conerr1');conerr1El.innerText=d.odriveControllerError1;conerr1El.className=d.odriveControllerError1==='0'?'value error-ok':'value error-critical';";
  h += "const ballEl=document.getElementById('ball');const ballStatus=document.getElementById('ball-status');";
  h += "if(d.ballState==='0'){ballEl.innerText='BALL NOT SHOT';ballStatus.className='item ball-idle'}";
  h += "else if(d.ballState==='1'){ballEl.innerText='SHOT - WAITING TO LEAVE';ballStatus.className='item ball-shot'}";
  h += "else if(d.ballState==='2'){ballEl.innerText='LEFT - WAITING RETURN';ballStatus.className='item ball-waiting'}";
  h += "else{ballEl.innerText='STATE '+d.ballState;ballStatus.className='item ball-idle'}";
  h += "const flapsEl=document.getElementById('flaps');const flapsStatus=document.getElementById('flaps-status');";
  h += "if(d.flapsOpen==='1'){flapsEl.innerText='OPEN';flapsStatus.className='item shooting-active'}else{flapsEl.innerText='CLOSED';flapsStatus.className='item shooting-inactive'}";
  h += "const shootEl=document.getElementById('shooting');const shootStatus=document.getElementById('shoot-status');";
  h += "if(d.isOdriveShooting==='1'){shootEl.innerText='ACTIVE';shootStatus.className='item shooting-active'}else{shootEl.innerText='INACTIVE';shootStatus.className='item shooting-inactive'}";
  h += "document.getElementById('lu').innerText=d.lastUpdate};";
  h += "w.onclose=()=>setTimeout(c,1000)}c();";
  h += "</script></body></html>";
  return h;
}

void sendStatus(uint8_t id) {
  StaticJsonDocument<1200> j;
  j["frontLeft"] = frontLeft;
  j["frontRight"] = frontRight;
  j["rearLeft"] = rearLeft;
  j["rearRight"] = rearRight;
  j["encoder1"] = encoder1;
  j["encoder2"] = encoder2;
  j["encoder3"] = encoder3;
  j["robotX"] = robotX;
  j["robotY"] = robotY;
  j["robotRotation"] = robotRotation;
  j["ballState"] = ballState;
  j["flapsOpen"] = flapsOpen;
  j["isOdriveShooting"] = isOdriveShooting;
  j["odriveVbusVoltage"] = odriveVbusVoltage;
  j["odriveVelocity0"] = odriveVelocity0;
  j["odriveVelocity1"] = odriveVelocity1;
  j["odriveCurrent0"] = odriveCurrent0;
  j["odriveCurrent1"] = odriveCurrent1;
  j["odriveError0"] = odriveError0;
  j["odriveError1"] = odriveError1;
  j["odriveAxisError0"] = odriveAxisError0;
  j["odriveAxisError1"] = odriveAxisError1;
  j["odriveEncoderError0"] = odriveEncoderError0;
  j["odriveEncoderError1"] = odriveEncoderError1;
  j["odriveControllerError0"] = odriveControllerError0;
  j["odriveControllerError1"] = odriveControllerError1;
  j["lastUpdate"] = String((millis() - lastUpdate) / 1000) + "s ago";
  String out;
  serializeJson(j, out);
  ws.sendTXT(id, out);
}

void wsEvent(uint8_t id, WStype_t t, uint8_t*, size_t) {
  if (t == WStype_CONNECTED) {
    Serial.printf("[WS] Client connected: %u\n", id);
    sendStatus(id);
  }
}

void handleRoot() {
  server.send(200, "text/html", getPage());
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 19, 21);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();

  ws.begin();
  ws.onEvent(wsEvent);

  Serial.println("Web server started at http://" + WiFi.localIP().toString());
}

void loop() {
  server.handleClient();
  ws.loop();

  if (Serial1.available()) {
    String d = Serial1.readStringUntil('\n');
    d.trim();
    if (d.startsWith("<") && d.endsWith(">")) {
      d = d.substring(1, d.length() - 1);
      int i[20], p = 0;
      for (int k = 0; k < 20; k++) {
        i[k] = d.indexOf(',', p);
        p = i[k] + 1;
      }

      frontLeft = d.substring(0, i[0]);
      frontRight = d.substring(i[0] + 1, i[1]);
      rearLeft = d.substring(i[1] + 1, i[2]);
      rearRight = d.substring(i[2] + 1, i[3]);
      encoder1 = d.substring(i[3] + 1, i[4]);
      encoder2 = d.substring(i[4] + 1, i[5]);
      encoder3 = d.substring(i[5] + 1, i[6]);
      isOdriveShooting = d.substring(i[6] + 1, i[7]);
      ballState = d.substring(i[7] + 1, i[8]);
      flapsOpen = d.substring(i[8] + 1, i[9]);
      odriveVbusVoltage = d.substring(i[9] + 1, i[10]);
      odriveVelocity0 = d.substring(i[10] + 1, i[11]);
      odriveVelocity1 = d.substring(i[11] + 1, i[12]);
      odriveCurrent0 = d.substring(i[12] + 1, i[13]);
      odriveCurrent1 = d.substring(i[13] + 1, i[14]);
      odriveAxisError0 = d.substring(i[14] + 1, i[15]);
      odriveAxisError1 = d.substring(i[15] + 1, i[16]);
      odriveEncoderError0 = d.substring(i[16] + 1, i[17]);
      odriveEncoderError1 = d.substring(i[17] + 1, i[18]);
      odriveControllerError0 = d.substring(i[18] + 1, i[19]);
      odriveControllerError1 = d.substring(i[19] + 1);
      lastUpdate = millis();

      for (uint8_t c = 0; c < ws.connectedClients(); c++) {
        sendStatus(c);
      }
    }
  }
}
