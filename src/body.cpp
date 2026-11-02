/*
 * body.cpp
 * 
 * Este arquivo contém a implementação dos métodos da classe 'RigidBody' no motor físico Cyclone.
 * Ele gerencia o estado dinâmico e cinemático de corpos rígidos 3D, incluindo conversões de 
 * coordenadas, atualização de tensores de inércia no mundo, integração de orientação com 
 * quatérnions e aplicação de forças e torques em pontos específicos.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/body.h>
#include <memory.h>
#include <assert.h>
#include <cmath>

namespace cyclone {

/*
 * =========================================================
 * MÉTODOS AUXILIARES DE TRANSFORMAÇÃO E INÉRCIA
 * =========================================================
 */

// Função auxiliar para transformar o tensor de inércia do espaço local para o espaço do mundo
static inline void transformInertiaTensor(
    Matrix3 &iitWorld,
    const Quaternion &q,
    const Matrix3 &iitBody,
    const Matrix4 &rotmat
) {
    real t4 = rotmat.data[0] * iitBody.data[0] +
              rotmat.data[1] * iitBody.data[3] +
              rotmat.data[2] * iitBody.data[6];
    real t9 = rotmat.data[0] * iitBody.data[1] +
              rotmat.data[1] * iitBody.data[4] +
              rotmat.data[2] * iitBody.data[7];
    real t14 = rotmat.data[0] * iitBody.data[2] +
               rotmat.data[1] * iitBody.data[5] +
               rotmat.data[2] * iitBody.data[8];
    real t28 = rotmat.data[4] * iitBody.data[0] +
               rotmat.data[5] * iitBody.data[3] +
               rotmat.data[6] * iitBody.data[6];
    real t33 = rotmat.data[4] * iitBody.data[1] +
               rotmat.data[5] * iitBody.data[4] +
               rotmat.data[6] * iitBody.data[7];
    real t38 = rotmat.data[4] * iitBody.data[2] +
               rotmat.data[5] * iitBody.data[5] +
               rotmat.data[6] * iitBody.data[8];
    real t52 = rotmat.data[8] * iitBody.data[0] +
               rotmat.data[9] * iitBody.data[3] +
               rotmat.data[10] * iitBody.data[6];
    real t57 = rotmat.data[8] * iitBody.data[1] +
               rotmat.data[9] * iitBody.data[4] +
               rotmat.data[10] * iitBody.data[7];
    real t62 = rotmat.data[8] * iitBody.data[2] +
               rotmat.data[9] * iitBody.data[5] +
               rotmat.data[10] * iitBody.data[8];

    iitWorld.data[0] = t4 * rotmat.data[0] + t9 * rotmat.data[1] + t14 * rotmat.data[2];
    iitWorld.data[1] = t4 * rotmat.data[4] + t9 * rotmat.data[5] + t14 * rotmat.data[6];
    iitWorld.data[2] = t4 * rotmat.data[8] + t9 * rotmat.data[9] + t14 * rotmat.data[10];
    iitWorld.data[3] = t28 * rotmat.data[0] + t33 * rotmat.data[1] + t38 * rotmat.data[2];
    iitWorld.data[4] = t28 * rotmat.data[4] + t33 * rotmat.data[5] + t38 * rotmat.data[6];
    iitWorld.data[5] = t28 * rotmat.data[8] + t33 * rotmat.data[9] + t38 * rotmat.data[10];
    iitWorld.data[6] = t52 * rotmat.data[0] + t57 * rotmat.data[1] + t62 * rotmat.data[2];
    iitWorld.data[7] = t52 * rotmat.data[4] + t57 * rotmat.data[5] + t62 * rotmat.data[6];
    iitWorld.data[8] = t52 * rotmat.data[8] + t57 * rotmat.data[9] + t62 * rotmat.data[10];
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE RIGIDBODY
 * =========================================================
 */

void RigidBody::calculateDerivedData() {
    // Normaliza a orientação para evitar distorções numéricas do quatérnion
    orientation.normalize();

    // Constrói a matriz de transformação afim 3x4 a partir da posição e orientação
    transformMatrix.setOrientationAndPos(orientation, position);

    // Transforma o tensor de inércia inverso para o espaço do mundo
    transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
}

void RigidBody::integrate(real duration) {
    // Se a massa inversa for zero, o corpo é estático e não se move
    if (inverseMass <= 0.0f) return;

    assert(duration > 0.0f);

    // 1. Calcula a aceleração linear com base nas forças acumuladas
    lastFrameAcceleration = acceleration;
    lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

    // 2. Calcula a aceleração angular com base nos torques acumulados e no tensor de inércia no mundo
    Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

    // 3. Atualiza as velocidades linear e angular
    velocity.addScaledVector(lastFrameAcceleration, duration);
    rotation.addScaledVector(angularAcceleration, duration);

    // 4. Aplica amortecimento (drag) linear e angular
    velocity *= real_pow(damping, duration);
    rotation *= real_pow(angularDamping, duration);

    // 5. Atualiza a posição linear
    position.addScaledVector(velocity, duration);

    // 6. Atualiza a orientação no espaço usando a taxa de rotação do quatérnion
    orientation.addScaledVector(rotation, duration);

    // 7. Recalcula matrizes de transformação e tensores derivados
    calculateDerivedData();

    // 8. Limpa os acumuladores de força e torque para o próximo quadro
    clearAccumulators();
}

void RigidBody::setMass(const real mass) {
    assert(mass != 0.0f);
    inverseMass = 1.0f / mass;
}

real RigidBody::getMass() const {
    if (inverseMass == 0.0f) {
        return ((real)INF_MASS);
    } else {
        return 1.0f / inverseMass;
    }
}

void RigidBody::setInverseMass(const real inverseMass) {
    this->inverseMass = inverseMass;
}

real RigidBody::getInverseMass() const {
    return inverseMass;
}

void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor) {
    inverseInertiaTensor.setInverse(inertiaTensor);
}

void RigidBody::setPosition(const Vector3 &position) {
    this->position = position;
}

void RigidBody::getPosition(Vector3 *position) const {
    *position = this->position;
}

Vector3 RigidBody::getPosition() const {
    return position;
}

void RigidBody::setOrientation(const Quaternion &orientation) {
    this->orientation = orientation;
    this->orientation.normalize();
}

Quaternion RigidBody::getOrientation() const {
    return orientation;
}

void RigidBody::setVelocity(const Vector3 &velocity) {
    this->velocity = velocity;
}

Vector3 RigidBody::getVelocity() const {
    return velocity;
}

void RigidBody::setRotation(const Vector3 &rotation) {
    this->rotation = rotation;
}

Vector3 RigidBody::getRotation() const {
    return rotation;
}

void RigidBody::clearAccumulators() {
    forceAccum.clear();
    torqueAccum.clear();
}

void RigidBody::addForce(const Vector3 &force) {
    forceAccum += force;
}

void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point) {
    // Converte o ponto do espaço do mundo para o espaço local do corpo
    Vector3 pt = point;
    pt -= position;

    forceAccum += force;
    torqueAccum += pt % force; // Produto vetorial gera torque
}

void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point) {
    // Converte ponto local para o espaço do mundo e aplica a força
    Vector3 pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);
}

void RigidBody::addTorque(const Vector3 &torque) {
    torqueAccum += torque;
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const {
    return transformMatrix.transform(point);
}

Vector3 RigidBody::getPointInBodySpace(const Vector3 &point) const {
    return transformMatrix.transformInverse(point);
}

Matrix4 RigidBody::getTransform() const {
    return transformMatrix;
}

Matrix3 RigidBody::getInverseInertiaTensorWorld() const {
    return inverseInertiaTensorWorld;
}

} // namespace cyclone
