/*
Implements encoder.h
*/
#include "encoder.h"

#include <pigpiod_if2.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

/**** Encoder Class Implementation ****/
Encoder::Encoder(int LApin, int LBpin, int RApin, int RBpin) :
                        m_pins{LApin, LBpin, RApin, RBpin}  {
    // Initialize pigpio daemon connection
    int m_pi = pigpio_start(NULL, NULL);
    if (m_pi < 0) {
        // failed
        std::cout << "Unable to connect to pigpio daemon! Pausing...";
        while (1) {};
    }
    std::cout << "Connected to pigpio daemon.";

    // set the encoder pins as inputs with pull-up
    for (int i{0}; i < 4; ++i) {
        set_mode(m_pi, m_pins[i], PI_INPUT);
        set_pull_up_down(m_pi, m_pins[i], PI_PUD_UP);
    }

    // initalize readings
    m_lastLA = gpio_read(m_pi, LApin);
    m_lastLB = gpio_read(m_pi, LBpin);
    m_lastRA = gpio_read(m_pi, RApin);
    m_lastRB = gpio_read(m_pi, RBpin);

    // prepare callbacks
    m_cbids[0] = callback_ex(m_pi, LApin, EITHER_EDGE, Encoder::LAchange, this);
    m_cbids[1] = callback_ex(m_pi, LBpin, EITHER_EDGE, Encoder::LBchange, this);
    m_cbids[2] = callback_ex(m_pi, RApin, EITHER_EDGE, Encoder::RAchange, this);
    m_cbids[3] = callback_ex(m_pi, RBpin, EITHER_EDGE, Encoder::RBchange, this);
}

void Encoder::shutdown() {
    // cancel callback functions
    std::cout << "Cancelling encoder callbacks...";
    for (int i{0}; i < 4; ++i) {
        callback_cancel(m_cbids[i]);
    }

    // wait til everything is done before stopping I/O
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    pigpio_stop(m_pi);
}

void Encoder::LAchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *encObj) {
    Encoder* enc = static_cast<Encoder*>(encObj);
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

void Encoder::LBchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *encObj) {
    Encoder* enc = static_cast<Encoder*>(encObj);
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

void Encoder::RAchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *encObj) {
    Encoder* enc = static_cast<Encoder*>(encObj);
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

void Encoder::RBchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *encObj) {
    std::cout << "RBchange";
    Encoder* enc = static_cast<Encoder*>(encObj);
    if (level == 0) {
        // RB FALLING
        if (!enc->m_lastRA)
            ++(enc->m_rcount);
        else
            --(enc->m_rcount);

        enc->m_lastRB = false;

    } else if (level == 1) {
        // RB RISING
        if (enc->m_lastRA)
            ++(enc->m_rcount);
        else
            --(enc->m_rcount);

        enc->m_lastRB = true;
    }
}

volatile bool out{0};
void ctrlc(int sig) {
    out = 1;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, ctrlc);

    // Create an encoder object
    Encoder enc = Encoder();

    std::cout << "Running...";

    while (1) {
        printf("Encoders: Left %d, Right %d\n", enc.getLeftCount(), enc.getRightCount());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (out) {
            break;
        }
    }

    // cleanup
    enc.shutdown();

    return 1;
}
