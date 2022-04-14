/*
C++ Port of the encoder reader library.
*/
#ifndef ATLAS_ENCODER_H
#define ATLAS_ENCODER_H

#include <pigpio.h>
#include <iostream>
#include <time.h>

// Encoder class
class Encoder {
public:
    Encoder(int LApin = 24, int LBpin = 25, int RApin = 23, int RBpin = 22);

    void shutdown();

    // encoder count commands
    int getLeftCount() { return m_lcount; }
    int getRightCount() { return m_rcount; }

private:
    int m_pi{};
    int m_cbids[4]{};

    static void LAchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *enc);
    static void LBchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *enc);
    static void RAchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *enc);
    static void RBchange(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *enc);

    bool m_lastLA{};
    bool m_lastLB{};
    bool m_lastRA{};
    bool m_lastRB{};

    int m_lcount{};
    int m_rcount{};

    int m_pins[4]{};
};

#endif
