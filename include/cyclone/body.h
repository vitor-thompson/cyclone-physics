#pragma once

#include <cyclone/core.h>

namespace cyclone {
    /**
     * @brief A rigid body is the basic simulation object in the physics core.
     */
    class RigidBody
    {
    public:
        // --- PROPRIEDADES LINEARES ---
        real inverseMass;
        Vector3 position;
        Vector3 velocity;

        // --- PROPRIEDADES ROTACIONAIS ---
        Quaternion orientation;
        Vector3 rotation;
        
        /**
         * @brief Holds the inverse of the body’s inertia tensor.
         * @details Dado no espaço local do corpo (body space).
         */
        Matrix3 inverseInertiaTensor;

        // --- DADOS DERIVADOS (CACHE) ---
        Matrix4 transformMatrix;

        /**
         * @brief Calculates internal data from state data.
         */
        void calculateDerivedData();

        /**
         * @brief Sets the inertia tensor for the rigid body.
         * @param inertiaTensor The tensor to set (will be inverted and stored).
         */
        void setInertiaTensor(const Matrix3 &inertiaTensor);
    };
}
