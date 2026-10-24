#pragma once

#include <cyclone/core.h>

namespace cyclone {
    class RigidBody
    {
    public:
        // --- PROPRIEDADES LINEARES ---
        real inverseMass;
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;         // Aceleração linear atual
        Vector3 lastFrameAcceleration;// Aceleração do quadro anterior
        Vector3 forceAccum;
        real linearDamping;           // Amortecimento linear

        // --- PROPRIEDADES ROTACIONAIS ---
        Quaternion orientation;
        Vector3 rotation;             // Velocidade angular
        Vector3 torqueAccum;
        real angularDamping;          // Amortecimento angular
        
        Matrix3 inverseInertiaTensor;
        Matrix3 inverseInertiaTensorWorld;

        // --- DADOS DERIVADOS (CACHE) ---
        Matrix4 transformMatrix;

        // --- FUNÇÕES DE SUPORTE ---
        
        bool hasFiniteMass() const
        {
            return inverseMass > 0.0f;
        }

        real getMass() const
        {
            if (inverseMass == 0.0f) {
                return ((real)1e10); // Valor muito alto para massa infinita
            } else {
                return ((real)1.0) / inverseMass;
            }
        }

        Vector3 getPointInWorldSpace(const Vector3 &point) const
        {
            return transformMatrix * point;
        }

        void calculateDerivedData();
        void setInertiaTensor(const Matrix3 &inertiaTensor);
        void clearAccumulators();

        void addForce(const Vector3 &force);
        void addForceAtPoint(const Vector3 &force, const Vector3 &point);
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        void addTorque(const Vector3 &torque);

        /**
         * @brief Executa a integração física do corpo rígido para o intervalo de tempo dado.
         */
        void integrate(real duration);
    };
}
