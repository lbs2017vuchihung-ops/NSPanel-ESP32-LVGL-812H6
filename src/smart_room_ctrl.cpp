#include "smart_room_ctrl.h"
#include "espnow_manager.h"

struct SystemSnapshot
{
    bool valid = false;
    bool roomLightOn = false;
    bool acPower[4] = {false, false, false, false}; // Chỉ số 1..3
};

static bool s_roomLightOn = false;
static bool s_masterBreakerOn = false;
static bool s_yellowLightOn = false;
static AcUnitState s_ac[4]; // Chỉ số 1..3
static SystemSnapshot s_snapshot;
static uint8_t s_lastLedState = 0xFF;
static UiStateSyncCallback s_uiSyncCb = nullptr;

static bool anyAcRunning()
{
    return s_ac[1].power || s_ac[2].power || s_ac[3].power;
}

static uint8_t computeTrafficLedState()
{
    if (s_masterBreakerOn)
        return 3; // 11: ĐỎ (CB Tổng)
    if (s_roomLightOn)
        return 2; // 10: XANH (Đèn phòng)
    if (s_yellowLightOn)
        return 1; // 01: VÀNG (Cảnh báo AC)
    return 0;     // 00: TẮT HẲN
}

static bool syncTrafficLed(bool forceSend = false)
{
    const uint8_t targetState = computeTrafficLedState();
    if (!forceSend && targetState == s_lastLedState)
        return true;

    s_lastLedState = targetState;
    return espnowSendTrafficCommand(targetState);
}

static void notifyUi()
{
    if (s_uiSyncCb)
        s_uiSyncCb(s_roomLightOn, s_masterBreakerOn, s_yellowLightOn, s_ac);
}

void roomCtrlInit(UiStateSyncCallback uiSyncCb)
{
    s_uiSyncCb = uiSyncCb;
    s_roomLightOn = false;
    s_masterBreakerOn = false;
    s_yellowLightOn = false;
    s_snapshot.valid = false;
    s_lastLedState = 0xFF;
}

bool roomCtrlSetLight(bool on)
{
    if (s_masterBreakerOn)
    {
        if (on)
        {
            Serial.println("[CTRL] Tu choi bat Den Phong: CB Tong dang bat!");
            notifyUi();
            return false;
        }
        s_roomLightOn = false;
        return true;
    }

    s_roomLightOn = on;

    // Bật đèn xanh -> tắt cảnh báo vàng | Tắt đèn xanh khi còn AC chạy -> tự bật đèn vàng
    if (on)
        s_yellowLightOn = false;
    else
        s_yellowLightOn = anyAcRunning();

    const bool ok = syncTrafficLed(false);
    notifyUi();
    return ok;
}

bool roomCtrlSetYellowLight(bool on)
{
    // Không cho bật đèn vàng khi CB Tổng hoặc Đèn xanh đang chiếm quyền hiển thị
    if (s_masterBreakerOn || (s_roomLightOn && on))
    {
        notifyUi();
        return false;
    }

    s_yellowLightOn = on;
    const bool ok = syncTrafficLed(false);
    notifyUi();
    return ok;
}

bool roomCtrlSetAc(uint8_t acId, bool power, uint8_t temp, uint8_t fan, uint8_t swing)
{
    if (acId < 1 || acId > 3)
        return false;

    if (s_masterBreakerOn)
    {
        if (power)
        {
            Serial.printf("[CTRL] Tu choi bat May lanh #%u: CB Tong dang bat!\n", acId);
            notifyUi();
            return false;
        }
        s_ac[acId] = {false, temp, fan, swing};
        return true;
    }

    const bool wasRunning = anyAcRunning();
    s_ac[acId] = {power, temp, fan, swing};
    const bool nowRunning = anyAcRunning();

    if (!s_roomLightOn)
    {
        // Tự bật đèn vàng khi có máy lạnh mới chuyển từ Tắt -> Bật lúc đèn phòng đang tắt
        if (!wasRunning && power)
            s_yellowLightOn = true;
        // Tự tắt đèn vàng khi tất cả máy lạnh đều đã tắt
        else if (!nowRunning)
            s_yellowLightOn = false;
    }

    const bool ok = espnowSendAcCommand(acId, power, temp, fan, swing);
    syncTrafficLed(false);
    notifyUi();
    return ok;
}

bool roomCtrlSetMasterBreaker(bool on)
{
    if (on == s_masterBreakerOn)
        return true;

    if (on)
    {
        s_snapshot.valid = true;
        s_snapshot.roomLightOn = s_roomLightOn;
        for (uint8_t i = 1; i <= 3; i++)
            s_snapshot.acPower[i] = s_ac[i].power;

        s_masterBreakerOn = true;
        s_roomLightOn = false;
        s_yellowLightOn = false;
        syncTrafficLed(true);
        notifyUi();

        for (uint8_t i = 1; i <= 3; i++)
        {
            if (s_ac[i].power)
            {
                s_ac[i].power = false;
                notifyUi();
                vTaskDelay(pdMS_TO_TICKS(120));
                espnowSendAcCommand(i, false, s_ac[i].temp, s_ac[i].fan, s_ac[i].swing);
            }
        }
        return true;
    }
    else
    {
        s_masterBreakerOn = false;

        if (s_snapshot.valid)
        {
            s_roomLightOn = s_snapshot.roomLightOn;
            for (uint8_t i = 1; i <= 3; i++)
                s_ac[i].power = s_snapshot.acPower[i];

            s_yellowLightOn = (!s_roomLightOn && anyAcRunning());
            syncTrafficLed(true);
            notifyUi();

            for (uint8_t i = 1; i <= 3; i++)
            {
                if (s_snapshot.acPower[i])
                {
                    vTaskDelay(pdMS_TO_TICKS(120));
                    espnowSendAcCommand(i, true, s_ac[i].temp, s_ac[i].fan, s_ac[i].swing);
                }
            }
            s_snapshot.valid = false;
        }
        else
        { 
            syncTrafficLed(true);
            notifyUi();
        }
        return true;
    }
}

bool roomCtrlIsLightOn() { return s_roomLightOn; }
bool roomCtrlIsMasterBreakerOn() { return s_masterBreakerOn; }
bool roomCtrlIsYellowLightOn() { return s_yellowLightOn; }
AcUnitState roomCtrlGetAc(uint8_t acId) { return (acId >= 1 && acId <= 3) ? s_ac[acId] : AcUnitState{}; }