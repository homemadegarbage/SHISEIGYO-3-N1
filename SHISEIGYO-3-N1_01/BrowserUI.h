#pragma once

#include <Arduino.h>
#include <IPAddress.h>

void beginBrowserUI(const char* ssid, const char* pass, const IPAddress& ip, const IPAddress& subnet);
void handleBrowserClient();
