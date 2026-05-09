#include "WirelessCmd.h"
const char* ssid = "STAGE_WAGON_BK";
const char* password = "bachkhoatphcm";
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if(type == WStype_TEXT) {
        
        JsonDocument doc; // Tương thích ArduinoJson v7
        DeserializationError error = deserializeJson(doc, payload);
        if (error) return;

        // --- XỬ LÝ LỆNH STOP KHẨN CẤP ---
        if (doc["cmd"] == "STOP") {
            v_target = 0;
            current_cmd = "STOP";
            Serial.println(">>> EMERGENCY STOP!");
            return; 
        }

        // --- XỬ LÝ CHẾ ĐỘ MANUAL ---
        if (doc["mode"] == "MANUAL") {
            current_cmd = doc["cmd"].as<String>();
            v_target = doc["v"].as<float>();
            a_target = doc["a"].as<float>();
            dist_target = doc["dist"].as<float>();
            
            drive_dir = 1;

            if (current_cmd == "N") { steer_angle = 0.0; }
            else if (current_cmd == "S") { steer_angle = 0.0; drive_dir = -1; }
            else if (current_cmd == "NE") { steer_angle = 45.0; }
            else if (current_cmd == "SW") { steer_angle = 45.0; drive_dir = -1; }
            else if (current_cmd == "NW") { steer_angle = -45.0; }
            else if (current_cmd == "SE") { steer_angle = -45.0; drive_dir = -1; }
            else if (current_cmd == "E") { steer_angle = 90.0; }
            else if (current_cmd == "W") { steer_angle = -90.0; }
            else if (current_cmd == "CW") { steer_angle = 45.0; drive_dir = 1; }
            else if (current_cmd == "CCW") { steer_angle = 45.0; drive_dir = -1; }

            Serial.printf("MANUAL -> CMD: %s | Steer: %.1f | Dir: %d | V: %.2f\n", current_cmd.c_str(), steer_angle, drive_dir, v_target);
        }

        // --- XỬ LÝ CHẾ ĐỘ AUTO ---
        else if (doc["mode"] == "AUTO") {
            Serial.println(">>> New AUTO Path Received!");
            JsonArray path = doc["path"];
            for (JsonObject wp : path) {
                float x = wp["x"]; float y = wp["y"];
                Serial.printf("Waypoint: (%.2f, %.2f)\n", x, y);
            }
        }
    }
}
void setup(){
    Serial.begin(115200);
    
    // Khởi động Access Point
    WiFi.softAP(ssid, password);
    Serial.print("Wagon AP Started. IP: "); Serial.println(WiFi.softAPIP());

    // Trang chủ WebServer
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", index_html);
    });
    server.begin();

    // Khởi động WebSocket cổng 81
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}
void loop(){
    server.handleClient(); // Xử lý request từ trình duyệt
    webSocket.loop();      // Xử lý dữ liệu WebSocket

    // Gửi Feedback S1, S2 lên Web mỗi 500ms
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 500) {
        lastUpdate = millis();
        JsonDocument fb;
        fb["s1"] = random(-45, 45); 
        fb["s2"] = random(-45, 45);
        
        String response;
        serializeJson(fb, response);
        webSocket.broadcastTXT(response); // Gửi tới tất cả thiết bị
    }
}