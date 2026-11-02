/*
 * pfgen.cpp
 * 
 * Este arquivo contém a implementação dos geradores de força e do registro de forças
 * aplicadas a partículas no motor físico Cyclone (ParticleGravity, ParticleDrag, 
 * ParticleAnchoredSpring e ParticleForceRegistry).
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/pfgen.h>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLEGRAVITY
 * =========================================================
 */

ParticleGravity::ParticleGravity(const Vector3 &gravity)
    : gravity(gravity) {}

void ParticleGravity::updateForce(Particle *particle, real duration) {
    // Verifica se a partícula possui massa finita antes de aplicar força
    if (!particle->hasFiniteMass()) return;

    // Aplica a força de gravidade multiplicada pela massa da partícula (F = m * g)
    particle->addForce(gravity * particle->getMass());
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLEDRAG
 * =========================================================
 */

ParticleDrag::ParticleDrag(real k1, real k2)
    : k1(k1), k2(k2) {}

void ParticleDrag::updateForce(Particle *particle, real duration) {
    Vector3 force;
    particle->getVelocity(&force);

    // Calcula a magnitude da velocidade
    real dragCoeff = force.magnitude();
    
    // Calcula o efeito combinado de arrasto linear (k1) e quadrático (k2)
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

    // Aplica a força na direção oposta à velocidade
    force.normalize();
    force *= -dragCoeff;
    particle->addForce(force);
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLEANCHOREDSPRING
 * =========================================================
 */

ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 *anchor, real springConstant, real restLength)
    : anchor(anchor), springConstant(springConstant), restLength(restLength) {}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration) {
    Vector3 force;
    particle->getPosition(&force);

    // Calcula o vetor diferença entre a partícula e o ponto de ancoragem fixo
    force -= *anchor;

    // Calcula o comprimento atual da mola
    real magnitude = force.magnitude();

    // Calcula a magnitude da força elástica usando a Lei de Hooke: F = -k * (l - l0)
    real displacement = std::abs(magnitude - restLength);
    real forceMagnitude = displacement * springConstant;

    // Normaliza o vetor e aplica a força no sentido correto
    force.normalize();
    force *= -forceMagnitude;
    particle->addForce(force);
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE PARTICLEFORCEREGISTRY
 * =========================================================
 */

void ParticleForceRegistry::add(Particle *particle, ParticleForceGenerator *fg) {
    ParticleForceRegistration registration;
    registration.particle = particle;
    registration.fg = fg;
    registrations.push_back(registration);
}

void ParticleForceRegistry::remove(Particle *particle, ParticleForceGenerator *fg) {
    for (auto it = registrations.begin(); it != registrations.end(); ++it) {
        if (it->particle == particle && it->fg == fg) {
            registrations.erase(it);
            break;
        }
    }
}

void ParticleForceRegistry::clear() {
    registrations.clear();
}

void ParticleForceRegistry::updateForces(real duration) {
    for (auto i = registrations.begin(); i != registrations.end(); ++i) {
        i->fg->updateForce(i->particle, duration);
    }
}

} // namespace cyclone
