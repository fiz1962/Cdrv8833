#include "Cdrv8833.h"
#include "esp32-hal-gpio.h"

Cdrv8833::Cdrv8833() {
	// not initialized
	m_in1Pin = -1;
	m_in2Pin = -1;
	m_power = 0;
	m_swapDirection = false;
	m_decayMode = drv8833DecaySlow;
}

Cdrv8833::Cdrv8833(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection) {
	init(in1Pin, in2Pin, channel, swapDirection);
}

Cdrv8833::~Cdrv8833() {
	stop();
}

bool Cdrv8833::init(uint8_t in1Pin, uint8_t in2Pin, uint8_t channel, bool swapDirection) {
	if (channel > 15)
		return false;
	if ((m_in1Pin != -1) && (m_in2Pin != -1))
		stop();
	pinMode(in1Pin, OUTPUT);
	pinMode(in2Pin, OUTPUT);
	m_in1Pin = in1Pin;
	m_in2Pin = in2Pin;
	m_power = 0;
	m_swapDirection = swapDirection;
	m_channel = channel;
	m_decayMode = drv8833DecaySlow;
	#if ESP_IDF_VERSION_MAJOR < 5 
            ledcSetup(channel, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
  #endif
	return true;
}

bool Cdrv8833::move(int8_t power) {
	if (-1 == m_in1Pin)
		return false;
	if (-1 == m_in2Pin)
		return false;
	if (0 == power) {
		stop();
		return true;
	}
	if (power > 100)
		power = 100;
	if (power < -100)
		power = -100;
	m_power = power;

	if (m_swapDirection)
		power = -power;
	float value = (float)((1 << PWM_BIT_RESOLUTION) - 1) * ((float)abs(power))/100.0;
	uint32_t dutyCycle;

	if ((value - trunc(value)) < 0.5)
		dutyCycle = value;
	else
		dutyCycle = value + 1;

	if (drv8833DecaySlow == m_decayMode)
		dutyCycle = ((1 << PWM_BIT_RESOLUTION) - 1) - dutyCycle;

	if (power > 0) { // forward
		if (drv8833DecayFast == m_decayMode) {
			// forward fast decay
			#if ESP_IDF_VERSION_MAJOR < 5 
				ledcDetachPin(m_in2Pin);
				digitalWrite(m_in2Pin, LOW);
				ledcAttachPin(m_in1Pin, m_channel);
			#else
				ledcDetach(m_in2Pin);
				digitalWrite(m_in2Pin, LOW);
				ledcAttachChannel(m_in1Pin, PWM_FREQUENCY, PWM_BIT_RESOLUTION, m_channel);			    
			#endif
		}
		else {
			// forward slow decay
			#if ESP_IDF_VERSION_MAJOR < 5 
				ledcDetachPin(m_in1Pin);
				digitalWrite(m_in1Pin, HIGH);
				ledcAttachPin(m_in2Pin, m_channel);
			#else
				ledcDetach(m_in1Pin);
				digitalWrite(m_in1Pin, LOW);
				ledcAttachChannel(m_in2Pin, PWM_FREQUENCY, PWM_BIT_RESOLUTION, m_channel);			    
			#endif
		}
	}
	else { // reverse
		if (drv8833DecayFast == m_decayMode) {
			// reverse fast decay
			#if ESP_IDF_VERSION_MAJOR < 5 
				ledcDetachPin(m_in1Pin);
				digitalWrite(m_in1Pin, LOW);
				ledcAttachPin(m_in2Pin, m_channel);
			#else
				ledcDetach(m_in1Pin);
				digitalWrite(m_in1Pin, LOW);
				ledcAttachChannel(m_in2Pin, PWM_FREQUENCY, PWM_BIT_RESOLUTION, m_channel);			
			#endif
		}
		else {
			// reverse slow decay
			#if ESP_IDF_VERSION_MAJOR < 5 
				ledcDetachPin(m_in2Pin);
				digitalWrite(m_in2Pin, HIGH);
				ledcAttachPin(m_in1Pin, m_channel);
			#else
				ledcDetach(m_in2Pin);
				digitalWrite(m_in2Pin, LOW);
				ledcAttachChannel(m_in1Pin, PWM_FREQUENCY, PWM_BIT_RESOLUTION, m_channel);			
			#endif
		}
	}
	#if ESP_IDF_VERSION_MAJOR < 5 
	    ledcWrite(m_channel, dutyCycle);
	#else
            ledcWriteChannel(m_channel, dutyCycle);
        #endif
        
	return true;
}

bool Cdrv8833::stop() {
	if (-1 == m_in1Pin)
		return false;
	if (-1 == m_in2Pin)
		return false;
	#if ESP_IDF_VERSION_MAJOR < 5 
		ledcDetachPin(m_in1Pin);
		ledcDetachPin(m_in2Pin);
	#else
		ledcDetach(m_in1Pin);
		ledcDetach(m_in2Pin);	
	#endif
	digitalWrite(m_in1Pin, LOW);
	digitalWrite(m_in2Pin, LOW);
	m_power = 0;
	return true;
}

bool Cdrv8833::brake() {
	if (-1 == m_in1Pin)
		return false;
	if (-1 == m_in2Pin)
		return false;
	#if ESP_IDF_VERSION_MAJOR < 5 
		ledcDetachPin(m_in1Pin);
		ledcDetachPin(m_in2Pin);
	#else
		ledcDetach(m_in1Pin);
		ledcDetach(m_in2Pin);	
	#endif
	digitalWrite(m_in1Pin, HIGH);
	digitalWrite(m_in2Pin, HIGH);
	m_power = 0;
	return true;
}

void Cdrv8833::setDecayMode(drv8833DecayMode decayMode) {
	stop();
	m_decayMode = decayMode;
}

void Cdrv8833::setFrequency(uint32_t frequency) {
	stop();
	ledcChangeFrequency(m_channel, frequency, PWM_BIT_RESOLUTION);
}

void Cdrv8833::swapDirection(bool swapDirection) {
	stop();
	m_swapDirection = swapDirection;
}

