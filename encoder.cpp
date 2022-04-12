/*
Implements encoder.h
*/
#include "encoder.h"

#include <pigpio.h>
#include <iostream.h>
#include <thread.h>
#include <chrono.h>
#include <signal.h>

/**** Encoder Class Implementation ****/
Encoder::Encoder(int LApin, int LBpin, int RApin, int RBpin) {
    // Initialize pigpio daemon connection
    if (gpioInitialise() < 0) {
        // failed
        std::cout << "Unable to connect to pigpio daemon! Pausing...";
        while (1) {};
    }
    std::cout << "Connected to pigpio daemon.";

    // set the encoder pins as inputs with pull-up
    int pins[4] = [LApin, LBpin, RApin, RBpin];
    for (int i{0}; i < 4; ++i) {
        gpioSetMode(pins[i], PI_INPUT);
        gpioSetPullUpDown(pins[i], PI_PUD_UP);
    }

    // initalize readings
    m_lastLA = gpioRead(LApin);
    m_lastLB = gpioRead(LBpin);
    m_lastRA = gpioRead(RApin);
    m_lastRB = gpioRead(RBpin);

    // prepare callbacks
    gpioSetAlertFunc(LApin, Encoder::LAchange);
    gpioSetAlertFunc(LBpin, Encoder::LBchange);
    gpioSetAlertFunc(RApin, Encoder::RAchange);
    gpioSetAlertFunc(RBpin, Encoder::RBchange);
}

void Encoder::shutdown() {
    // cancel callback functions
    std::cout << "Cancelling encoder callbacks...";
    gpioSetAlertFunc(LApin, null);
    gpioSetAlertFunc(LBpin, null);
    gpioSetAlertFunc(RApin, null);
    gpioSetAlertFunc(RBpin, null);

    // wait til everything is done before stopping I/O
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    gpioTerminate();
}

void Encoder::LAchange(int gpio, int level, uint32_t tick) {
    if (level == 0) {
        // LA FALLING
        if (m_lastLB)
            ++m_lcount;
        else:
            --m_lcount;

        m_lastLA = false;

    } else if (level == 1) {
        // LA RISING
        if (!m_lastLB)
            ++m_lcount;
        else:
            --m_lcount;

        m_lastLA = true;
    }
}

void Encoder::LBchange(int gpio, int level, uint32_t tick) {
    if (level == 0) {
        // LB FALLING
        if (!m_lastLA)
            ++m_lcount;
        else:
            --m_lcount;

        m_lastLB = false;

    } else if (level == 1) {
        // LB RISING
        if (m_lastLA)
            ++m_lcount;
        else:
            --m_lcount;

        m_lastLB = true;
    }
}

void Encoder::RAchange(int gpio, int level, uint32_t tick) {
    if (level == 0) {
        // RA FALLING
        if (m_lastRB)
            ++m_rcount;
        else:
            --m_rcount;

        m_lastRA = false;

    } else if (level == 1) {
        // RA RISING
        if (!m_lastRB)
            ++m_rcount;
        else:
            --m_rcount;

        m_lastRA = true;
    }
}

void Encoder::LBchange(int gpio, int level, uint32_t tick) {
    if (level == 0) {
        // RB FALLING
        if (!m_lastRA)
            ++m_rcount;
        else:
            --m_rcount;

        m_lastRB = false;

    } else if (level == 1) {
        // RB RISING
        if (m_lastRA)
            ++m_rcount;
        else:
            --m_rcount;

        m_lastRB = true;
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
        printf("Encoders: Left %d, Right %d", enc.getLeftCount(), enc.getRightCount())
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (out) {
            break;
        }
    }

    // cleanup
    enc.shutdown();
}
