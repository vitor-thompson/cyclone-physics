/*
 * pcontacts.cpp
 * 
 * Este arquivo contém a implementação dos métodos de detecção, tratamento de velocidades
 * e resolução de interpenetração para contatos e restrições de partículas (ParticleContact
 * e ParticleContactResolver).
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/pcontacts.h>
#include <assert.h>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLECONTACT
 * =========================================================
 */

void ParticleContact::resolve(real duration) {
    // 1. Resolve os efeitos devidos à velocidade (impulso de ressalto/restituição)
    resolveVelocity(duration);

    // 2. Resolve os efeitos geométricos devidos à interpenetração (afundamento)
    resolveInterpenetration(duration);
}

real ParticleContact::calculateSeparatingVelocity() const {
    Vector3 relativeVelocity = particle[0]->getVelocity();
    if (particle[1]) {
        relativeVelocity -= particle[1]->getVelocity();
    }
    return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real duration) {
    // Calcula a velocidade separadora inicial entre os corpos
    real separatingVelocity = calculateSeparatingVelocity();

    // Se a velocidade separadora for positiva ou zero, as partículas já estão se afastando
    if (separatingVelocity > 0) {
        return;
    }

    // Calcula a nova velocidade separadora após o impacto considerando a restituição
    real newSepVelocity = -separatingVelocity * restitution;

    // Verifica o efeito de aceleração gerada por forças externas durante o quadro atual
    Vector3 accCausedVelocity = particle[0]->getAcceleration();
    if (particle[1]) {
        accCausedVelocity -= particle[1]->getAcceleration();
    }
    real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

    // Se houver velocidade gerada por aceleração colaborando para a separação, remove-a do cálculo de restituição
    if (accCausedSepVelocity < 0) {
        newSepVelocity += restitution * accCausedSepVelocity;
        if (newSepVelocity < 0) {
            newSepVelocity = 0;
        }
    }

    real deltaVelocity = newSepVelocity - separatingVelocity;

    // Aplica o impulso de acordo com a massa inversa de cada partícula
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) {
        totalInverseMass += particle[1]->getInverseMass();
    }

    // Se a massa inversa total for zero, ambas possuem massa infinita e nenhum impulso pode ser aplicado
    if (totalInverseMass <= 0) return;

    // Calcula o impulso por unidade de massa inversa
    real impulse = deltaVelocity / totalInverseMass;
    Vector3 impulsePerIMass = contactNormal * impulse;

    // Aplica o impulso na primeira partícula
    particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass());

    // Aplica o impulso oposto na segunda partícula (se existir)
    if (particle[1]) {
        particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass());
    }
}

void ParticleContact::resolveInterpenetration(real duration) {
    // Se não houver interpenetração mensurável, não há o que corrigir
    if (penetration <= 0) return;

    // Calcula a massa inversa total
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) {
        totalInverseMass += particle[1]->getInverseMass();
    }

    if (totalInverseMass <= 0) return;

    // Encontra o movimento por unidade de massa inversa
    Vector3 movePerIMass = contactNormal * (-penetration / totalInverseMass);

    // Armazena o movimento aplicado (útil para depuração ou restrições)
    particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
    if (particle[1]) {
        particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
    } else {
        particleMovement[1] = Vector3(0, 0, 0);
    }

    // Aplica o deslocamento corretivo de posição na primeira partícula
    particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);

    // Aplica o deslocamento corretivo na segunda partícula (se existir)
    if (particle[1]) {
        particle[1]->setPosition(particle[1]->getPosition() + particleMovement[1]);
    }
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLECONTACTRESOLVER
 * =========================================================
 */

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
    : iterations(iterations) {}

void ParticleContactResolver::setIterations(unsigned iterations) {
    this->iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration) {
    if (numContacts == 0) return;
    if (iterations == 0) return;

    iterationsUsed = 0;
    while (iterationsUsed < iterations) {
        // Encontra o contato com a maior severidade (pior interpenetração ou maior velocidade de aproximação)
        unsigned maxIndex = numContacts;
        real maxVal = (real)0.0;
        
        for (unsigned i = 0; i < numContacts; i++) {
            real sepVel = contactArray[i].calculateSeparatingVelocity();
            if (sepVel < maxVal || contactArray[i].penetration > 0) {
                maxVal = sepVel;
                maxIndex = i;
            }
        }

        // Se nenhum contato crítico foi encontrado, encerra a resolução antecipadamente
        if (maxIndex == numContacts) {
            break;
        }

        // Resolve o contato mais severo encontrado
        contactArray[maxIndex].resolve(duration);
        iterationsUsed++;
    }
}

} // namespace cyclone
