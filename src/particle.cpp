/*
 * particle.cpp
 * 
 * Este arquivo contém a implementação dos métodos da classe 'Particle' no motor físico Cyclone.
 * Ele lida com a integração de Euler semi-implícita, atualização de posições, velocidades,
 * amortecimento (damping) e aplicação de forças acumuladas por quadro.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/particle.h>
#include <assert.h>

namespace cyclone {

void Particle::integrate(real duration) {
    // Se a massa inversa for zero, a partícula tem massa infinita (estática/inamovível) e não se move.
    if (inverseMass <= 0.0f) return;

    assert(duration > 0.0f);

    // 1. Atualiza a posição linear com base na velocidade atual
    position.addScaledVector(velocity, duration);

    // 2. Calcula a aceleração resultante combinando a aceleração fixa (ex: gravidade) com forças aplicadas
    Vector3 resultingAcceleration = acceleration;
    resultingAcceleration.addScaledVector(forceAccum, inverseMass);

    // 3. Atualiza a velocidade linear usando a aceleração resultante e o intervalo de tempo
    velocity.addScaledVector(resultingAcceleration, duration);

    // 4. Aplica o amortecimento (drag/damping) para simular atrito com o ar e evitar instabilidade numérica
    velocity *= real_pow(damping, duration);

    // 5. Limpa o acumulador de forças para o próximo quadro
    clearAccumulator();
}

void Particle::setMass(const real mass) {
    assert(mass != 0.0f);
    inverseMass = 1.0f / mass;
}

real Particle::getMass() const {
    if (inverseMass == 0.0f) {
        return ((real)INF_MASS); // Massa infinita
    } else {
        return 1.0f / inverseMass;
    }
}

void Particle::setInverseMass(const real inverseMass) {
    this->inverseMass = inverseMass;
}

real Particle::getInverseMass() const {
    return inverseMass;
}

bool Particle::hasFiniteMass() const {
    return inverseMass >= 0.0f;
}

void Particle::setPosition(const Vector3 &position) {
    this->position = position;
}

void Particle::setPosition(const real x, const real y, const real z) {
    position.x = x;
    position.y = y;
    position.z = z;
}

void Particle::getPosition(Vector3 *position) const {
    *position = this->position;
}

Vector3 Particle::getPosition() const {
    return position;
}

void Particle::setVelocity(const Vector3 &velocity) {
    this->velocity = velocity;
}

void Particle::setVelocity(const real x, const real y, const real z) {
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

void Particle::getVelocity(Vector3 *velocity) const {
    *velocity = this->velocity;
}

Vector3 Particle::getVelocity() const {
    return velocity;
}

void Particle::setAcceleration(const Vector3 &acceleration) {
    this->acceleration = acceleration;
}

void Particle::setAcceleration(const real x, const real y, const real z) {
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}

void Particle::getAcceleration(Vector3 *acceleration) const {
    *acceleration = this->acceleration;
}

Vector3 Particle::getAcceleration() const {
    return acceleration;
}

void Particle::setDamping(const real damping) {
    this->damping = damping;
}

real Particle::getDamping() const {
    return damping;
}

void Particle::clearAccumulator() {
    forceAccum.x = 0;
    forceAccum.y = 0;
    forceAccum.z = 0;
}

void Particle::addForce(const Vector3 &force) {
    forceAccum += force;
}

} // namespace cyclone
