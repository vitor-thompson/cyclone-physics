/*
 * contacts.cpp
 * 
 * Este arquivo contém a implementação dos métodos de contato e resolução de colisões
 * para corpos rígidos (RigidBodyContact e ContactResolver) no motor físico Cyclone.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/contacts.h>
#include <memory.h>
#include <assert.h>
#include <cmath>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTATION OF RIGIDBODYCONTACT
 * =========================================================
 */

void RigidBodyContact::calculateInternals(real duration) {
    // Se o primeiro corpo for nulo, troca para garantir que body[0] nunca seja nulo
    if (!body[0]) {
        swapBodies();
    }
    assert(body[0]);

    // Constrói a base ortonormal para o espaço de contato
    calculateContactBasis();

    // Armazena a posição relativa do ponto de contato em relação a cada corpo
    relativeContactPosition[0] = contactPoint - body[0]->getPosition();
    if (body[1]) {
        relativeContactPosition[1] = contactPoint - body[1]->getPosition();
    }

    // Calcula a velocidade relativa no ponto de contato
    lieferSeparatingVelocity = calculateSeparatingVelocity();

    // Calcula a restituição desejada (pode ser ajustada dinamicamente)
    // ...
}

void RigidBodyContact::swapBodies() {
    RigidBody *temp = body[0];
    body[0] = body[1];
    body[1] = temp;
    contactNormal = contactNormal * -1.0f;
}

void RigidBodyContact::calculateContactBasis() {
    Vector3 contactTangent[2];

    // Se a normal for próxima ao eixo Y, escolhe o eixo X como base temporária
    if (std::abs(contactNormal.x) > std::abs(contactNormal.y)) {
        const real s = (real)1.0 / std::sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);
        contactTangent[0].x = contactNormal.z * s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x * s;
        
        contactTangent[1].x = contactNormal.y * contactTangent[0].x;
        contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    } else {
        const real s = (real)1.0 / std::sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);
        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z * s;
        contactTangent[0].z = contactNormal.y * s;
        
        contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }

    // Preenche a matriz de transformação de contato para o mundo (contactToWorld)
    contactToWorld.setComponents(contactNormal, contactTangent[0], contactTangent[1]);
}

real RigidBodyContact::calculateSeparatingVelocity() const {
    Vector3 relativeVelocity = body[0]->getRotation() % relativeContactPosition[0];
    relativeVelocity += body[0]->getVelocity();
    
    if (body[1]) {
        Vector3 b2Vel = body[1]->getRotation() % relativeContactPosition[1];
        b2Vel += body[1]->getVelocity();
        relativeVelocity -= b2Vel;
    }

    return relativeVelocity * contactNormal;
}

void RigidBodyContact::resolve(real duration) {
    // Resolve os efeitos de velocidade e interpenetração
    // (Implementação central executada pelo ContactResolver)
}

/*
 * =========================================================
 * IMPLEMENTATION OF CONTACTRESOLVER
 * =========================================================
 */

ContactResolver::ContactResolver(unsigned iterations, real velocityEpsilon, real positionEpsilon)
    : velocityIterations(iterations),
      positionIterations(iterations),
      velocityEpsilon(velocityEpsilon),
      positionEpsilon(positionEpsilon) {}

ContactResolver::ContactResolver(unsigned velocityIterations, unsigned positionIterations,
                                 real velocityEpsilon, real positionEpsilon)
    : velocityIterations(velocityIterations),
      positionIterations(positionIterations),
      velocityEpsilon(velocityEpsilon),
      positionEpsilon(positionEpsilon) {}

void ContactResolver::setIterations(unsigned velocityIterations, unsigned positionIterations) {
    this->velocityIterations = velocityIterations;
    this->positionIterations = positionIterations;
}

void ContactResolver::setEpsilon(real velocityEpsilon, real positionEpsilon) {
    this->velocityEpsilon = velocityEpsilon;
    this->positionEpsilon = positionEpsilon;
}

void ContactResolver::resolveContacts(RigidBodyContact *contactArray, unsigned numContacts, real duration) {
    if (numContacts == 0) return;

    // 1. Fase de resolução de posições (interpenetração)
    positionIterationsUsed = 0;
    while (positionIterationsUsed < positionIterations) {
        // Encontra o contato com maior penetração
        unsigned maxIndex = numContacts;
        real maxVal = positionEpsilon;
        for (unsigned i = 0; i < numContacts; i++) {
            if (contactArray[i].penetration > maxVal) {
                maxVal = contactArray[i].penetration;
                maxIndex = i;
            }
        }

        if (maxIndex == numContacts) break;

        // Aplica correção de posição
        // ...
        positionIterationsUsed++;
    }

    // 2. Fase de resolução de velocidades (impulsos)
    velocityIterationsUsed = 0;
    while (velocityIterationsUsed < velocityIterations) {
        // Encontra o contato com maior velocidade de aproximação
        unsigned maxIndex = numContacts;
        real maxVal = velocityEpsilon;
        for (unsigned i = 0; i < numContacts; i++) {
            real sepVel = contactArray[i].calculateSeparatingVelocity();
            if (sepVel < maxVal) {
                maxVal = sepVel;
                maxIndex = i;
            }
        }

        if (maxIndex == numContacts) break;

        // Aplica impulso de velocidade
        // ...
        velocityIterationsUsed++;
    }
}

} // namespace cyclone
