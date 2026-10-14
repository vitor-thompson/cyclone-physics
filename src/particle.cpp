#include <cassert>
#include <cyclone/particle.h>

namespace cyclone {
    void Particle::integrate(real duration)
    {
        assert(duration > 0.0f);

        // 1. Atualiza a posição linear
        position.addScaledVector(velocity, duration);

        // 2. Calcula a aceleração resultante (incluindo a gravidade/aceleração base + forças acumuladas)
        Vector3 resultingAcc = acceleration;
        resultingAcc.addScaledVector(forceAccum, inverseMass);

        // 3. Atualiza a velocidade linear a partir da aceleração resultante
        velocity.addScaledVector(resultingAcc, duration);

        // 4. Aplica o amortecimento (damping / arrasto)
        velocity *= real_pow(damping, duration);

        // 5. Limpa as forças acumuladas para o próximo quadro
        clearAccumulator();
    }

    void Particle::clearAccumulator()
    {
        forceAccum = Vector3(0, 0, 0);
    }

    void Particle::addForce(const Vector3 &force)
    {
        forceAccum.addScaledVector(force, 1.0f); // Soma vetorial da nova força no acumulador
    }
}
