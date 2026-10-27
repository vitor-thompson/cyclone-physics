#include <cyclone/collide_coarse.h>
#include <cyclone/precision.h>

using namespace cyclone;

// ============================================================================
// --- BOUNDING SPHERE ---
// ============================================================================

BoundingSphere::BoundingSphere(const Vector3 &center, real radius)
    : center(center), radius(radius) {}

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &two)
{
    Vector3 centerOffset = two.center - one.center;
    real distanceSquared = centerOffset.squareMagnitude();
    real radiusDiff = two.radius - one.radius;

    // Verifica se uma esfera engloba totalmente a outra
    if (radiusDiff * radiusDiff >= distanceSquared)
    {
        if (one.radius > two.radius)
        {
            center = one.center;
            radius = one.radius;
        }
        else
        {
            center = two.center;
            radius = two.radius;
        }
    }
    else
    {
        // Caso contrário, calcula uma esfera intermediária que abrange ambas
        real distance = real_sqrt(distanceSquared);
        radius = (distance + one.radius + two.radius) * 0.5f;
        center = one.center;
        if (distance > 0.0f)
        {
            center += centerOffset * ((radius - one.radius) / distance);
        }
    }
}

bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
    real distanceSquared = (center - other->center).squareMagnitude();
    real radiusSum = radius + other->radius;
    
    // Compara a distância ao quadrado para evitar o cálculo lento de raiz quadrada
    return distanceSquared < (radiusSum * radiusSum);
}

real BoundingSphere::getGrowth(const BoundingSphere &other) const
{
    // Cria uma esfera unificada temporária e mede o aumento de volume aproximado
    BoundingSphere newSphere(*this, other);
    
    real oldVolume = radius * radius * radius;
    real newVolume = newSphere.radius * newSphere.radius * newSphere.radius;
    
    return newVolume - oldVolume;
}
