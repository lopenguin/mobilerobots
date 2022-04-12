/*
Implements encoder.h
*/
#include "encoder.h"

#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

/**** Encoder Class Implementation ****/
Encoder::Encoder(int LApin, int LBpin, int RApin, int RBpin) :
                        m_pins{LApin, LBpin, RApin, RBpin}  {
    // Initialize pigpio daemon connection
    if (gpioInitialise() < 0) {
        // failed
        std::cout << "Unable to connect to pigpio daemon! Pausing...";
        while (1) {};
    }
    std::cout << "Connected to pigpio daemon.";

    // set the encoder pins as inputs with pull-up
    for (int i{0}; i < 4; ++i) {
        gpioSetMode(m_pins[i], PI_INPUT);
        gpioSetPullUpDown(m_pins[i], PI_PUD_UP);
    }

    // initalize readings
    m_lastLA = gpioRead(LApin);
    m_lastLB = gpioRead(LBpin);
    m_lastRA = gpioRead(RApin);
    m_lastRB = gpioRead(RBpin);

    // prepare callbacks
    gpioSetAlertFuncEx(LApin, Encoder::LAchange, this);
    gpioSetAlertFuncEx(LBpin, Encoder::LBchange, this);
    gpioSetAlertFuncEx(RApin, Encoder::RAchange, this);
    gpioSetAlertFuncEx(RBpin, Encoder::RBchange, this);
}

void Encoder::shutdown() {
    // cancel callback functions
    std::cout << "Cancelling encoder callbacks...";
    for (int i{0}; i < 4; ++i) {
        gpioSetAlertFunc(m_pins[i], NULL);
    }

    // wait til everything is done before stopping I/O
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    gpioTerminate();
}

void Encoder::LAchange(int gpio, int level, uint32_t tick, void *encObj) {
    Encoder* enc = static_cast<*Encoder>(encObj);
    if (level == 0) {
        // LA FALLING
        if (enc->m_lastLB)
            ++enc->m_lcount;
        else
            --enc->m_lcount;

        enc->m_lastLA = false;

    } else if (level == 1) {
        // LA RISING
        if (!enc->m_lastLB)
            ++enc->m_lcount;
        else
            --enc->m_lcount;

        enc->m_lastLA = true;
    }
}

void Encoder::LBchange(int gpio, int level, uint32_t tick, void *enc) {
    if (level == 0) {
        // LB FALLING
        if (!enc->m_lastLA)
            ++enc->m_lcount;
        else
            --enc->m_lcount;

        enc->m_lastLB = false;

    } else if (level == 1) {
        // LB RISING
        if (enc->m_lastLA)
            ++enc->m_lcount;
        else
            --enc->m_lcount;

        enc->m_lastLB = true;
    }
}

void Encoder::RAchange(int gpio, int level, uint32_t tick, void *enc) {
    if (level == 0) {
        // RA FALLING
        if (enc->m_lastRB)
            ++enc->m_rcount;
        else
            --enc->m_rcount;

        enc->m_lastRA = false;

    } else if (level == 1) {
        // RA RISING
        if (!enc->m_lastRB)
            ++enc->m_rcount;
        else
            --enc->m_rcount;

        enc->m_lastRA = true;
    }
}

void Encoder::RBchange(int gpio, int level, uint32_t tick, void *enc) {
    if (level == 0) {
        // RB FALLING
        if (!enc->m_lastRA)
            ++enc->m_rcount;
        else
            --enc->m_rcount;

        enc->m_lastRB = false;

    } else if (level == 1) {
        // RB RISING
        if (enc->m_lastRA)
            ++enc->m_rcount;
        else
            --enc->m_rcount;

        enc->m_lastRB = true;
    }
}

volatile bool out{0};
void exit(int sig) {
    out = 1;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, exit);

    // Create an encoder object
    Encoder enc = Encoder();

    std::cout << "Running...";

    while (1) {
        printf("Encoders: Left %d, Right %d", enc.getLeftCount(), enc.getRightCount());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (out) {
            break;
        }
    }

    // cleanup
    enc.shutdown();

    return 1;
}
