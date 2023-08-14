/*
 * hcsr04_driver.h
 *
 *  Created on: 10 сент. 2022 г.
 *      Author: Ierixon-HP
 */

#ifndef INC_HCSR04_DRIVER_H_
#define INC_HCSR04_DRIVER_H_
#include "stm32f4xx_hal.h"

/**
 * Driver for HC-SR04.
 */
class HCSR04Driver {
    public:
        HCSR04Driver() = default;

        /**
         * Initialize timers.
         *
         * Note: pins should be configured separatly.
         */
        void init(TIM_TypeDef* tim, uint32_t triggerChannel, uint32_t echoChannel, float soundSpeed = 343.0f);

         /**
         * This method should be invoked when timer update event occurs
         */
        void _acknowledgeTimerUpdate();

        /**
         * This method should be invoked when channel input capture event occurs
         */
        void _acknowledgeChannelCapture();

        /**
         * Get distance in meters.
         *
         * Negative values will be returned if there is no object before sensor. */
        float getDistance();

        /**
         * Get distance in the seconds.
         *
         * Negative values will be returned if there is no object before sensor. */
         float getDistanceInSeconds();

    private:
        enum EchoPulseState {
            NOT_STARTED,
            STARTED,
            ENDED,
        };

        float soundSpeed;

        // base timer structure and channel codes
        TIM_HandleTypeDef hTim;
        uint32_t triggerChannel;
        uint32_t echoChannel;

        // start and end of echo pulse
        uint32_t echoPulseStart;
        uint32_t echoPulseEnd;

        // echo pulse state
        EchoPulseState echoPulseState;

        // calculated echo delay
        int32_t echoDelay;

         // control parameters
        static const uint32_t PROBING_FREQUNCY = 20; // 20 Hz
        static const uint32_t COUNTER_FREQUNCY = 100000; // 100 kHz
        static const uint32_t TRIGGER_PULSE_LEN = 10; // 10 us
        static const uint32_t INTERRUPT_PRIOIRY = 8;
        static const uint32_t INTERRUPT_SUBPRIOIRY = 8;
};




#endif /* INC_HCSR04_DRIVER_H_ */
