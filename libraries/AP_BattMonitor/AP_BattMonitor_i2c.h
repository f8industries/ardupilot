#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define ADS1112_addr 0x48
#define ADS1112_left_addr 0x4A
#define ADS1112_bus 0
#define curr_reg_set 0x4C		//0b01001100	AIN0
#define volt_reg_set 0x6C		//0b01101100 	AIN1


class AP_BattMonitor_i2c : public AP_BattMonitor_Backend
{
public:

    /// Constructor
	AP_BattMonitor_i2c(AP_BattMonitor &mon,
					   AP_BattMonitor::BattMonitor_State &mon_state,
					   AP_BattMonitor_Params &params,
					   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    void read_data(uint8_t reg, uint16_t* data);

    void init(void) override;

    virtual void timer(void) = 0;

protected:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t switch_ch = 0;

};



class AP_BattMonitor_ads1112 : public AP_BattMonitor_i2c
{
public:
	AP_BattMonitor_ads1112(AP_BattMonitor &mon,
	                             AP_BattMonitor::BattMonitor_State &mon_state,
	                             AP_BattMonitor_Params &params,
	                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

	void timer(void) override;
};
