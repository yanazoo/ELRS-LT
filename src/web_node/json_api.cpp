#include <Arduino.h>
#include <ArduinoJson.h>
#include <new>
#include "json_api.h"
#include "data_model.h"

const char* activeName(int slot) {
    int ri = activePilots[slot];
    return (ri >= 0 && ri < rosterCount) ? roster[ri].name : "---";
}

int activeSlotOf(int ri) {
    for (int s = 0; s < MAX_ACTIVE; s++) if (activePilots[s] == ri) return s;
    return -1;
}

String rosterJson() {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < rosterCount; i++) {
        JsonObject o = arr.add<JsonObject>();
        o["id"]         = i;
        o["name"]       = roster[i].name;
        o["yomi"]       = roster[i].yomi;
        o["uid"]        = roster[i].hasUid ? [&](){
                            char u[18]; uidToStr(roster[i].uid, u); return String(u);
                          }() : String("");
        o["activeSlot"] = activeSlotOf(i);
        o["enter"]      = rosterCal[i].enterRssi;
        o["exit"]       = rosterCal[i].exitRssi;
    }
    String s; serializeJson(doc, s); return s;
}

String activeJson() {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int s = 0; s < MAX_ACTIVE; s++) {
        JsonObject o = arr.add<JsonObject>();
        int ri = activePilots[s];
        o["slot"]      = s;
        o["rosterIdx"] = ri;
        o["name"]      = activeName(s);
        o["yomi"]      = (ri >= 0 && ri < rosterCount) ? roster[ri].yomi : "";
        if (ri >= 0 && ri < rosterCount) {
            o["enter"] = rosterCal[ri].enterRssi;
            o["exit"]  = rosterCal[ri].exitRssi;
        }
    }
    String s; serializeJson(doc, s); return s;
}

String scanJson() {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < scanMacCount; i++) {
        JsonObject o = arr.add<JsonObject>();
        o["mac"] = scanMacs[i].mac;
        o["rssi"] = scanMacs[i].rssi;
        o["ts"]   = scanMacs[i].ts;
    }
    String s; serializeJson(doc, s); return s;
}

// Max accepted request-body size. All API bodies are small JSON; this caps the
// per-request allocation so a rogue/huge Content-Length cannot exhaust the heap
// on the AsyncTCP task.
#define MAX_BODY_BYTES 4096

void handleBody(AsyncWebServerRequest* req,
                uint8_t* data, size_t len, size_t index, size_t total,
                std::function<void(AsyncWebServerRequest*, const char*)> cb) {
    struct BodyBuf { char* buf; size_t total; };
    if (index == 0) {
        if (total > MAX_BODY_BYTES) {
            req->send(413, "application/json", R"({"error":"body too large"})");
            return;  // _tempObject stays null; later chunks are ignored below
        }
        char* b = new (std::nothrow) char[total + 1];
        if (!b) { req->send(500, "application/json", R"({"error":"oom"})"); return; }
        auto* nb = new (std::nothrow) BodyBuf{ b, total };
        if (!nb) { delete[] b; req->send(500, "application/json", R"({"error":"oom"})"); return; }
        req->_tempObject = nb;
    }
    auto* bb = reinterpret_cast<BodyBuf*>(req->_tempObject);
    if (!bb) return;
    memcpy(bb->buf + index, data, len);
    if (index + len == total) {
        bb->buf[total] = '\0';
        cb(req, bb->buf);
        delete[] bb->buf; delete bb; req->_tempObject = nullptr;
    }
}
