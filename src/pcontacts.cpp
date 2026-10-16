#include <cyclone/pcontacts.h>

namespace cyclone {

    // --- ParticleContact ---
    void ParticleContact::resolve(real duration)
    {
        resolveVelocity(duration);
        resolveInterpenetration(duration);
    }

    real ParticleContact::calculateSeparatingVelocity() const
    {
        Vector3 relativeVelocity = particle[0]->getVelocity();
        if (particle[1]) {
            relativeVelocity -= particle[1]->getVelocity();
        }
        return relativeVelocity * contactNormal;
    }

    void ParticleContact::resolveVelocity(real duration)
    {
        real separatingVelocity = calculateSeparatingVelocity();

        if (separatingVelocity > 0) {
            return;
        }

        real newSepVelocity = -separatingVelocity * restitution;

        Vector3 accCausedVelocity = particle[0]->getAcceleration();
        if (particle[1]) {
            accCausedVelocity -= particle[1]->getAcceleration();
        }
        real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

        if (accCausedSepVelocity < 0)
        {
            newSepVelocity += restitution * accCausedSepVelocity;
            if (newSepVelocity < 0) {
                newSepVelocity = 0;
            }
        }

        real deltaVelocity = newSepVelocity - separatingVelocity;

        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) {
            totalInverseMass += particle[1]->getInverseMass();
        }

        if (totalInverseMass <= 0) return;

        real impulse = deltaVelocity / totalInverseMass;
        Vector3 impulsePerIMass = contactNormal * impulse;

        particle[0]->setVelocity(
            particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass()
        );

        if (particle[1]) {
            particle[1]->setVelocity(
                particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass()
            );
        }
    }

    void ParticleContactResolver::resolveInterpenetration(real duration)
    {
        // Nota: A correção de interpenetração posicional pode ser integrada ou separada. 
        // No algoritmo principal, iteramos os contatos por severidade.
    }

    // Para manter a coerência com a função membro anterior, ajustamos se necessário:
    void ParticleContact::resolveInterpenetration(real duration)
    {
        if (penetration <= 0) return;

        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) {
            totalInverseMass += particle[1]->getInverseMass();
        }

        if (totalInverseMass <= 0) return;

        Vector3 movePerIMass = contactNormal * (-penetration / totalInverseMass);

        particle[0]->setPosition(
            particle[0]->getPosition() + movePerIMass * particle[0]->getInverseMass()
        );

        if (particle[1]) {
            particle[1]->setPosition(
                particle[1]->getPosition() + movePerIMass * -particle[1]->getInverseMass()
            );
        }
    }

    // --- ParticleContactResolver ---
    ParticleContactResolver::ParticleContactResolver(unsigned iterations)
        : iterations(iterations), iterationsUsed(0) {}

    void ParticleContactResolver::setIterations(unsigned iterations)
    {
        this->iterations = iterations;
    }

    void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration)
    {
        if (numContacts == 0) return;

        iterationsUsed = 0;
        while (iterationsUsed < iterations)
        {
            // 1. Encontra o contato com a menor velocidade de separação (mais severo)
            real max = 0;
            unsigned maxIndex = numContacts;
            for (unsigned i = 0; i < numContacts; i++)
            {
                real sepVel = contactArray[i].calculateSeparatingVelocity();
                if (sepVel < max)
                {
                    max = sepVel;
                    maxIndex = i;
                }
            }

            // Se nenhum contato possui velocidade negativa (estão separados ou parados), encerra
            if (maxIndex == numContacts || max >= 0) {
                break;
            }

            // 2. Resolve o contato mais severo
            contactArray[maxIndex].resolve(duration);

            iterationsUsed++;
        }
    }
}
