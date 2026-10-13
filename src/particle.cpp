#include <cassert>
#include <cyclone/particle.h>

namespace cyclone {
    void Particle::integrate(real duration)
    {
        // Garante que o intervalo de tempo seja estritamente positivo
        assert(duration > 0.0f);

        // 1. Atualiza a posição linear baseada na velocidade atual
        position.addScaledVector(velocity, duration);

        // 2. Calcula a aceleração resultante a partir da aceleração base e das forças acumuladas
        Vector3 resultingAcc = acceleration;
        resultingAcc.addScaledVector(forceAccum, inverseMass);

        // 3. Atualiza a velocidade linear a partir da aceleração resultante
        velocity.addScaledVector(resultingAcc, duration);

        // 4. Aplica o amortecimento (drag/damping) ajustado pelo tempo
        velocity *= real_pow(damping, duration);

        // 5. Limpa o acumulador de forças para o próximo quadro
        // (Nota: limpamos zerando os componentes do vetor de força acumulada)
        forceAccum = Vector3(0, 0, 0);
    }
}
