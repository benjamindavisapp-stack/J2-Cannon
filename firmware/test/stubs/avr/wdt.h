#pragma once
#define WDTO_500MS 5
void wdt_enable(int);
void wdt_disable();
void wdt_reset();
