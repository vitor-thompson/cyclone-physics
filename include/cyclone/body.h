#pragma once

#include <cyclone/core.h>

namespace cyclone {
    /**
     * @brief A rigid body is the basic simulation object in the physics core.
     */
    class RigidBody
    {
    public:
        /**
         * @brief Holds the inverse of the mass of the rigid body.
         */
        real inverseMass;

        /**
         * @brief Holds the linear position of the rigid body in world space.
         */
        Vector3 position;

        /**
         * @brief Holds the angular orientation of the rigid body in world space.
         */
        Quaternion orientation;

        /**
         * @brief Holds the linear velocity of the rigid body in world space.
         */
        Vector3 velocity;

        /**
         * @brief Holds the angular velocity, or rotation, of the rigid body in world space.
         */
        Vector3 rotation;

        /**
         * @brief Holds a transform matrix for converting body space into world space and vice versa.
         */
        Matrix4 transformMatrix;

        /**
         * @brief Calculates internal data from state data.
         */
        void calculateDerivedData();
    };
}
