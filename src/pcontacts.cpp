#include <cyclone/pcontacts.h>

namespace cyclone {

    void ParticleContact::resolve(real duration)
    {
        resolveVelocity(duration);
    }

    real ParticleContact::calculateSeparatingVelocity() const
    {
        Vector3 relativeVelocity = particle[0]->getVelocity();
        if (particle[1]) {
            relativeVelocity -= particle[1]->getVelocity();
        }
        return relativeVelocity * contactNormal; // Produto escalar
    }

    void ParticleContact::resolveVelocity(real duration)
    {
        // 1. Encontra a velocidade relativa na direção da normal de contato
        real separatingVelocity = calculateSeparatingVelocity();

        // 2. Se a velocidade já for positiva, os objetos estão se afastar ou parados - nenhum impulso necessário
        if (separatingVelocity > 0)
        {
            return;
        }

        // 3. Calcula a nova velocidade de separação desejada usando o coeficiente de restituição
        real newSepVelocity = -separatingVelocity * restitution;
        real deltaVelocity = newSepVelocity - separatingVelocity;

        // 4. Soma a massa inversa total dos objetos envolvidos
        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) {
            totalInverseMass += particle[1]->getInverseMass();
        }

        // Se todos os objetos tiverem massa infinita, o impulso não tem efeito
        if (totalInverseMass <= 0) return;

        // 5. Calcula o impulso necessário
        real impulse = deltaVelocity / totalInverseMass;
        Vector3 impulsePerIMass = contactNormal * impulse;

        // 6. Aplica os impulsos nas direções corretas proporcionalmente à massa inversa
        particle[0]->setVelocity(
            particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass()
        );

        if (particle[1])
        {
            // A segunda partícula vai na direção oposta
            particle[1]->setVelocity(
                particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass()
            );
        }
    }
}
