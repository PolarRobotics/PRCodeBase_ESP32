#pragma once

#ifndef CFG_MGR_H
#define CFG_MGR_H

#include <Arduino.h>
#include <Preferences.h>

// #include <Utilities/BotTypes.h>
#include <PolarRobotics.h>

#if CFG_WRITABLE == 1
  #define DEFAULT_WRITABLE 1
#else
  #define DEFAULT_WRITABLE 0
#endif

class ConfigManager {
  private:
    bool writable = DEFAULT_WRITABLE;
    
    bot_config_t* config;
    
    // Members of Config
    uint8_t bot_name_index;
    BotType bot_type;
    MotorType motor_type;

    // Write Function
    bool write(bot_config_t *cfg);
  protected:
    Preferences preferences;
  public:
    ConfigManager();
    // ConfigManager(bool writable = false);
    ~ConfigManager();
    void read();
    int botIndex();
    BotType getBotType();
    MotorType getMotorType();
    // int GearRatio()
    const char * toString();
    bool setConfig(uint8_t botindex);
    bool setConfig(uint8_t botindex, BotType bottype, MotorType motortype);
};

#endif // CFG_MGR_H