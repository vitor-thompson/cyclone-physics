namespace cyclone {

    /**
     * Representa um contato entre dois corpos rígidos no motor.
     */
    class Contact
    {
    public:
        /** Ponto do contato em coordenadas globais. */
        Vector3 contactPoint;

        /** Direção do contato (aponta do primeiro corpo para o segundo). */
        Vector3 contactNormal;

        /** Profundidade de interpenetração no ponto de contato. */
        real penetration;
    };

}
