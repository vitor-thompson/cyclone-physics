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
         * @brief Holds the inverse of the body’s inertia tensor in body space.
         */
        Matrix3 inverseInertiaTensor;

        // --- DADOS DERIVADOS (CACHE) ---
        Matrix4 transformMatrix;

        /**
         * @brief Holds the inverse of the body's inertia tensor in world space.
         * @details Derivado a cada quadro a partir de inverseInertiaTensor e da orientação.
         */
        Matrix3 inverseInertiaTensorWorld;

        /**
         * @brief Calculates internal data from state data.
         */
        void calculateDerivedData();

        /**
         * @brief Sets the inertia tensor for the rigid body.
         */
        void setInertiaTensor(const Matrix3 &inertiaTensor);
    };
}
