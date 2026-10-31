/*
 * Implementação de Corpos Rígidos do motor Cyclone.
 * Atualizado com normalização de quatérnios e rastreamento de movimento por RWA.
 */

#include "cyclone/body.h"
#include <memory.h>
#include <assert.h>

namespace cyclone {

    static real g_sleepEpsilon = (real)0.01f;

    void setSleepEpsilon(real epsilon) { g_sleepEpsilon = epsilon; }
    real getSleepEpsilon() { return g_sleepEpsilon; }

    void RigidBody::calculateDerivedData()
    {
        orientation.normalize();
        transformMatrix.setOrientationAndPos(orientation, position);
        calculateInverseInertiaTensorWorld(&inverseInertiaTensorWorld, inverseInertiaTensor, transformMatrix);
    }

    void RigidBody::integrate(real duration)
    {
        if (!isAwake) return;

        lastFrameAcceleration = acceleration;
        lastFrameAcceleration += forceAccumulator * inverseMass;

        Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccumulator);

        velocity += lastFrameAcceleration * duration;
        rotation += angularAcceleration * duration;

        velocity *= real_pow(linearDamping, duration);
        rotation *= real_pow(angularDamping, duration);

        position += velocity * duration;
        orientation.addScaledVector(rotation, duration);

        // CORREÇÃO DE ESTABILIDADE (Cap. 16.1.1): Normalização do quatérnio
        orientation.normalize();

        calculateDerivedData();
        clearAccumulators(duration);
    }

    void RigidBody::clearAccumulators(real duration)
    {
        forceAccumulator = Vector3::ZERO;
        torqueAccumulator = Vector3::ZERO;
        lastFrameAcceleration = acceleration;

        // Sistema de Sono (Cap. 16.2.1 - RWA)
        if (canSleep)
        {
            real currentMotion = velocity.scalarProduct(velocity) + rotation.scalarProduct(rotation);
            real bias = real_pow(0.5f, duration);
            motion = bias * motion + (1.0f - bias) * currentMotion;

            if (motion > 10.0f * g_sleepEpsilon) {
                motion = 10.0f * g_sleepEpsilon;
            }

            if (motion < g_sleepEpsilon) {
                setAsleep(true);
            }
        }
    }

    void RigidBody::setAwake(const bool awake)
    {
        if (awake) {
            isAwake = true;
            motion = g_sleepEpsilon * 2.0f;
        } else {
            isAwake = false;
            velocity.clear();
            rotation.clear();
            motion = 0.0f;
        }
    }

    void RigidBody::setAsleep(bool asleep) { setAwake(!asleep); }

    void RigidBody::addForce(const Vector3 &force) { forceAccumulator += force; setAwake(true); }
    void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point) {
        Vector3 pt = point - position;
        forceAccumulator += force;
        torqueAccumulator += pt % force;
        setAwake(true);
    }
    void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point) {
        addForceAtPoint(force, transformMatrix.transform(point));
    }
    void RigidBody::addTorque(const Vector3 &torque) { torqueAccumulator += torque; setAwake(true); }

    void RigidBody::calculateInverseInertiaTensorWorld(
        Matrix3 *iitWorld, const Matrix3 &iitBody, const Matrix4 &rotmat
    )
    {
        real t4 = rotmat.data[0]*iitBody.data[0]+rotmat.data[1]*iitBody.data[3]+rotmat.data[2]*iitBody.data[6];
        real t9 = rotmat.data[0]*iitBody.data[1]+rotmat.data[1]*iitBody.data[4]+rotmat.data[2]*iitBody.data[7];
        real t14 = rotmat.data[0]*iitBody.data[2]+rotmat.data[1]*iitBody.data[5]+rotmat.data[2]*iitBody.data[8];
        real t28 = rotmat.data[4]*iitBody.data[0]+rotmat.data[5]*iitBody.data[3]+rotmat.data[6]*iitBody.data[6];
        real t33 = rotmat.data[4]*iitBody.data[1]+rotmat.data[5]*iitBody.data[4]+rotmat.data[6]*iitBody.data[7];
        real t38 = rotmat.data[4]*iitBody.data[2]+rotmat.data[5]*iitBody.data[5]+rotmat.data[6]*iitBody.data[8];
        real t52 = rotmat.data[8]*iitBody.data[0]+rotmat.data[9]*iitBody.data[3]+rotmat.data[10]*iitBody.data[6];
        real t57 = rotmat.data[8]*iitBody.data[1]+rotmat.data[9]*iitBody.data[4]+rotmat.data[10]*iitBody.data[7];
        real t62 = rotmat.data[8]*iitBody.data[2]+rotmat.data[9]*iitBody.data[5]+rotmat.data[10]*iitBody.data[8];

        iitWorld->data[0] = t4*rotmat.data[0]+t9*rotmat.data[1]+t14*rotmat.data[2];
        iitWorld->data[1] = t4*rotmat.data[4]+t9*rotmat.data[5]+t14*rotmat.data[6];
        iitWorld->data[2] = t4*rotmat.data[8]+t9*rotmat.data[9]+t14*rotmat.data[10];
        iitWorld->data[3] = t28*rotmat.data[0]+t33*rotmat.data[1]+t38*rotmat.data[2];
        iitWorld->data[4] = t28*rotmat.data[4]+t33*rotmat.data[5]+t38*rotmat.data[6];
        iitWorld->data[5] = t28*rotmat.data[8]+t33*rotmat.data[9]+t38*rotmat.data[10];
        iitWorld->data[6] = t52*rotmat.data[0]+t57*rotmat.data[1]+t62*rotmat.data[2];
        iitWorld->data[7] = t52*rotmat.data[4]+t57*rotmat.data[5]+t62*rotmat.data[6];
        iitWorld->data[8] = t52*rotmat.data[8]+t57*rotmat.data[9]+t62*rotmat.data[10];
    }

} // namespace cyclone
