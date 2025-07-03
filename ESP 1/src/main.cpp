#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char* ssid = "OPPO Reno6";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);

String frontLeft = "0", frontRight = "0", rearLeft = "0", rearRight = "0";
String encoder1 = "0", encoder2 = "0", encoder3 = "0";
String isOdriveShooting = "0";
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
  h += ".status{text-align:center;color:#6c757d;font-size:14px;margin-top:20px;padding:10px;background:#f8f9fa;border-radius:6px}";
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
  h += "<div class='section status-section'><h2>Shooting Status</h2>";
  h += "<div class='item' id='shoot-status'><div class='label'>ODrive Shooting</div><div class='value' id='shooting'>INACTIVE</div></div>";
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
  h += "const shootEl=document.getElementById('shooting');const shootStatus=document.getElementById('shoot-status');";
  h += "if(d.isOdriveShooting==='1'){shootEl.innerText='ACTIVE';shootStatus.className='item shooting-active'}else{shootEl.innerText='INACTIVE';shootStatus.className='item shooting-inactive'}";
  h += "document.getElementById('lu').innerText=d.lastUpdate};";
  h += "w.onclose=()=>setTimeout(c,1000)}c();";
  h += "</script></body></html>";
  return h;
}

void sendStatus(uint8_t id) {
  StaticJsonDocument<300> j;
  j["frontLeft"] = frontLeft;
  j["frontRight"] = frontRight;
  j["rearLeft"] = rearLeft;
  j["rearRight"] = rearRight;
  j["encoder1"] = encoder1;
  j["encoder2"] = encoder2;
  j["encoder3"] = encoder3;
  j["isOdriveShooting"] = isOdriveShooting;
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
      int i[7], p = 0;
      for (int k = 0; k < 7; k++) {
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
      isOdriveShooting = d.substring(i[6] + 1);
      lastUpdate = millis();

      for (uint8_t c = 0; c < ws.connectedClients(); c++) {
        sendStatus(c);
      }
    }
  }
}
