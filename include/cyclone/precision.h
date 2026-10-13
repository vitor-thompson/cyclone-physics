#pragma once

#include <cmath>

namespace cyclone {
    // Define se usamos float ou double para os cálculos do motor
    typedef float real;

    // Função de raiz quadrada compatível com a precisão escolhida
    inline real real_sqrt(real val) {
        return sqrtf(val);
    }

    // --- NOVO: Precisão para o operador de potência (usado no damping) ---
    #define real_pow powf
}
