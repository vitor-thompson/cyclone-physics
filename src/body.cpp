/*
 * Implementação do corpo rígido (RigidBody) do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#include "cyclone/bodies.h"

namespace cyclone {

    RigidBody::RigidBody()
        : inverseMass(0),
          linearDamping(0.95f),
          angularDamping(0.8f),
          isAwake(true),
          canSleep(true)
    {
        orientation.setIdentity();
        velocity.clear();
        rotation.clear();
        acceleration.clear();
        accelerationAtUpdate.clear();
        forceAccum.clear();
        torqueAccum.clear();
    }

    RigidBody::~RigidBody()
    {
    }

    void RigidBody::calculateDerivedData()
    {
        orientation.normalize();

        // Constrói a matriz de transformação 4x4 a partir de posição e orientação
        transformMatrix.setOrientationAndPos(orientation, position);

        // Calcula o tensor de inércia inverso no espaço do mundo
        transformMatrix.transformInertiaTensor(inverseInertiaTensorWorld, inverseInertiaTensor, transformMatrix);
    }

    void RigidBody::integrate(real duration)
    {
        if (!isAwake) return;

        // Calcula a aceleração linear final a partir das forças acumuladas
        acceleration = accelerationAtUpdate;
        acceleration.addScaledVector(forceAccum, inverseMass);

        // Calcula a aceleração angular a partir dos torques acumulados
        Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

        // Atualiza a velocidade linear (com amortecimento)
        velocity.addScaledVector(acceleration, duration);
        velocity *= real_pow(linearDamping, duration);

        // Atualiza a velocidade angular (com amortecimento)
        rotation.addScaledVector(angularAcceleration, duration);
        rotation *= real_pow(angularDamping, duration);

        // Atualiza a posição linear
        position.addScaledVector(velocity, duration);

        // Atualiza a orientação (usando o quatérnio de rotação)
        orientation.addScaledVector(rotation, duration);

        // Recalcula dados derivados (matriz e tensor de inércia)
        calculateDerivedData();

        // Limpa os acumuladores e guarda a aceleração para o próximo quadro
        clearAccumulators();
    }

    void RigidBody::clearAccumulators()
    {
        forceAccum.clear();
        torqueAccum.clear();
        accelerationAtUpdate = acceleration;
    }

    void RigidBody::addForce(const Vector3 &force)
    {
        forceAccum += force;
        isAwake = true;
    }

    void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point)
    {
        Vector3 pt = point;
        pt -= position;

        forceAccum += force;
        torqueAccum += pt % force;

        isAwake = true;
    }

    void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
    {
        Vector3 pt = transformMatrix.transform(point);
        addForceAtPoint(force, pt);
    }

    // Setters e Getters básicos
    void RigidBody::setMass(real mass)
    {
        if (mass <= 0) {
            inverseMass = 0;
        } else {
            inverseMass = (real)1.0f / mass;
        }
    }

    real RigidBody::getMass() const
    {
        if (inverseMass == 0) {
            return REAL_MAX;
        } else {
            return (real)1.0f / inverseMass;
        }
    }

    void RigidBody::setInverseMass(real inverseMass)
    {
        this->inverseMass = inverseMass;
    }

    real RigidBody::getInverseMass() const
    {
        return inverseMass;
    }

    bool RigidBody::hasFiniteMass() const
    {
        return inverseMass > 0;
    }

    void RigidBody::setPosition(const Vector3 &position)
    {
        this->position = position;
    }

    void RigidBody::getPosition(Vector3 *position) const
    {
        *position = this->position;
    }

    Vector3 RigidBody::getPosition() const
    {
        return position;
    }

    void RigidBody::setOrientation(const Quaternion &orientation)
    {
        this->orientation = orientation;
        this->orientation.normalize();
    }

    void RigidBody::getOrientation(Quaternion *orientation) const
    {
        *orientation = this->orientation;
    }

    Quaternion RigidBody::getOrientation() const
    {
        return orientation;
    }

    void RigidBody::setVelocity(const Vector3 &velocity)
    {
        this->velocity = velocity;
    }

    void RigidBody::getVelocity(Vector3 *velocity) const
    {
        *velocity = this->velocity;
    }

    Vector3 RigidBody::getVelocity() const
    {
        return velocity;
    }

    void RigidBody::setRotation(const Vector3 &rotation)
    {
        this->rotation = rotation;
    }

    void RigidBody::getRotation(Vector3 *rotation) const
    {
        *rotation = this->rotation;
    }

    Vector3 RigidBody::getRotation() const
    {
        return rotation;
    }

    void RigidBody::setAwake(bool awake)
    {
        if (awake) {
            isAwake = true;
        } else {
            isAwake = false;
            velocity.clear();
            rotation.clear();
        }
    }

} // namespace cyclone
