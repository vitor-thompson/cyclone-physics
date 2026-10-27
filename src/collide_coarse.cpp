namespace cyclone {

    struct BoundingSphere
    {
        Vector3 center;
        real radius;

        BoundingSphere(const Vector3 &center, real radius);
        BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

        /**
         * @brief Verifica se esta esfera se sobrepõe a outra.
         */
        bool overlaps(const BoundingSphere *other) const
        {
            real distanceSquared = (center - other->center).squareMagnitude();
            real radiusSum = radius + other->radius;
            
            // Compara a distância ao quadrado para evitar raízes quadradas lentas
            return distanceSquared < (radiusSum * radiusSum);
        }
    };

}
