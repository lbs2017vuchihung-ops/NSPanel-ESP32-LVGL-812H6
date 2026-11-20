#pragma once
#include <Arduino.h>

struct AcUnitState
{
    bool power = false;
    uint8_t temp = 25;
    uint8_t fan = 0;
    uint8_t swing = 0;
};

// Thêm tham số yellowLightOn để đồng bộ công tắc Đèn vàng trên UI
using UiStateSyncCallback = void (*)(bool roomLightOn, bool masterBreakerOn, bool yellowLightOn, const AcUnitState ac[4]);

void roomCtrlInit(UiStateSyncCallback uiSyncCb);

bool roomCtrlSetLight(bool on);
bool roomCtrlSetMasterBreaker(bool on);
bool roomCtrlSetYellowLight(bool on);
bool roomCtrlSetAc(uint8_t acId, bool power, uint8_t temp, uint8_t fan, uint8_t swing);

bool roomCtrlIsLightOn();
bool roomCtrlIsMasterBreakerOn();
bool roomCtrlIsYellowLightOn();
AcUnitState roomCtrlGetAc(uint8_t acId);