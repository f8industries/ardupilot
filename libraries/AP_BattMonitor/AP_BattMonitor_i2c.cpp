#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_i2c.h"
#include <AP_HAL/utility/functor.h>
#include <AP_Math/AP_Math.h>
#include <utility>

extern const AP_HAL::HAL& hal;

// Use git commit -am"MESSAGE"
//Constructor
AP_BattMonitor_i2c::AP_BattMonitor_i2c(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
        AP_BattMonitor_Backend(mon, mon_state, params),
	_dev(std::move(dev))
{
    // always healthy
    _state.healthy = true;
}

//initialise	10Hz callback
void
AP_BattMonitor_i2c::init()
{
    if (_dev) {
    	 _dev->register_periodic_callback(200000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_i2c::timer, void));
    }
}


// realised in cyclic callback
void AP_BattMonitor_i2c::read()
{

}

void AP_BattMonitor_i2c::read_data(uint8_t reg, uint16_t* data)
{
	uint8_t buff[3];	// buffer to hold results

	// read the appropriate register from the device
	if (!_dev->read_registers(reg, buff, sizeof(buff))) {
		*data = 7000; 	// magic number to indicate no reading
		return;
	}
	*data = (uint16_t)buff[0]<<8 | (uint16_t)buff[1];
}

/// return true if battery provides current info
bool AP_BattMonitor_i2c::has_current() const
{
    return true;//(_params.type() == AP_BattMonitor_Params::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
}


AP_BattMonitor_ads1112::AP_BattMonitor_ads1112(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_i2c(mon, mon_state, params, std::move(dev))
{
   // _pec_supported = true;
}

void AP_BattMonitor_ads1112::timer()
{
	//static uint8_t switch_ch = 0;     tworzy jedna zmienna dla wszystkich klas
	uint32_t tnow = AP_HAL::micros();
	uint16_t data=0;
	if(switch_ch){
	read_data(curr_reg_set, &data);
	}
	else{
		read_data(volt_reg_set, &data);
	}
	long rd = data;
	if (rd >= 32768) rd -= 65536l;
	double v = (double)rd * 2.048 / 32768.0;	// voltage on adc input

	if(switch_ch){// channel 1 -> current
		_state.current_amps = 	(v - _params._curr_amp_offset) * _params._curr_amp_per_volt;		//(v - hall_v_bias) * 82.255;

	}
	else{//channel 0 -> voltage
		_state.voltage = v * _params._volt_multiplier;												//22.21;
		//_dev->write_register(0, curr_reg_set, 0);
	}
	_state.last_time_micros = tnow;
	switch_ch = (switch_ch + 1) & 0b00000001;
}
