/*
 * Representação de Corpos Rígidos do motor Cyclone.
 * Atualizado com suporte a estabilidade de quatérnios e sistema de sono (Capítulo 16).
 */

#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

#include "precision.h"
#include "core.h"

namespace cyclone {

    class RigidBody
    {
    protected:
        real inverseMass;
        Vector3 position;
        Quaternion orientation;
        Vector3 velocity;
        Vector3 rotation;

        Matrix4 transformMatrix;
        Matrix3 inverseInertiaTensor;
        Matrix3 inverseInertiaTensorWorld;

        Vector3 acceleration;
        Vector3 lastFrameAcceleration;

        Vector3 forceAccumulator;
        Vector3 torqueAccumulator;

        // Atributos do Sistema de Sono (Capítulo 16.2.1)
        bool isAwake;
        bool canSleep;
        real motion;

        real linearDamping;
        real angularDamping;

    public:
        void integrate(real duration);
        void calculateDerivedData();
        void clearAccumulators(real duration);

        void setAwake(const bool awake = true);
        void setAsleep(bool asleep = true);
        bool getIsAwake() const { return isAwake; }

        void setAcceleration(const Vector3 &acc) { acceleration = acc; }
        Vector3 getAcceleration() const { return acceleration; }
        Vector3 getLastFrameAcceleration() const { return lastFrameAcceleration; }

        void setPosition(const Vector3 &pos) { position = pos; }
        Vector3 getPosition() const { return position; }

        void setOrientation(const Quaternion &ori) { orientation = ori; orientation.normalize(); }
        Quaternion getOrientation() const { return orientation; }

        void setVelocity(const Vector3 &vel) { velocity = vel; setAwake(true); }
        Vector3 getVelocity() const { return velocity; }

        void setRotation(const Vector3 &rot) { rotation = rot; setAwake(true); }
        Vector3 getRotation() const { return rotation; }

        real getInverseMass() const { return inverseMass; }
        bool hasFiniteMass() const { return inverseMass >= 0.0f; }

        void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const {
            *inverseInertiaTensor = inverseInertiaTensorWorld;
        }

        void addForce(const Vector3 &force);
        void addForceAtPoint(const Vector3 &force, const Vector3 &point);
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        void addTorque(const Vector3 &torque);

    private:
        static void calculateInverseInertiaTensorWorld(
            Matrix3 *iitWorld, const Matrix3 &iitBody, const Matrix4 &rotmat
        );
    };

} // namespace cyclone

#endif // CYCLONE_BODY_H
