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
        Vector3 forceAccum; // Acumulador de forças

        // --- PROPRIEDADES ROTACIONAIS ---
        Quaternion orientation;
        Vector3 rotation;
        Vector3 torqueAccum; // Acumulador de torques
        
        Matrix3 inverseInertiaTensor;
        Matrix3 inverseInertiaTensorWorld;

        // --- DADOS DERIVADOS (CACHE) ---
        Matrix4 transformMatrix;

        // --- FUNÇÕES DE TRANSFORMAÇÃO DE ESPAÇO ---
        
        /**
         * @brief Converts a point given in body space to world space.
         */
        Vector3 getPointInWorldSpace(const Vector3 &point) const
        {
            return transformMatrix * point;
        }

        // --- MÉTODOS DE INTEGRAÇÃO E ACUMULADORES ---
        
        void calculateDerivedData();
        void setInertiaTensor(const Matrix3 &inertiaTensor);
        
        /**
         * @brief Limpa os acumuladores de força e torque.
         */
        void clearAccumulators();

        /**
         * @brief Adiciona uma força ao centro de massa (coordenadas do mundo).
         */
        void addForce(const Vector3 &force);

        /**
         * @brief Adiciona uma força em um ponto específico do corpo (dado em coordenadas locais/body space).
         */
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);

        /**
         * @brief Adiciona uma força em um ponto específico (dado em coordenadas do mundo).
         */
        void addForceAtPoint(const Vector3 &force, const Vector3 &point);

        /**
         * @brief Executa a integração do corpo rígido para um determinado intervalo de tempo.
         */
        void integrate(real duration);
    };
}
