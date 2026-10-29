// 2. Proporção do movimento linear e angular
    real inverseInertia = 1.0f / totalInertia;
    
    real linearMove[2];
    real angularMove[2];

    linearMove[0] = penetration * linearInertia[0] * inverseInertia;
    linearMove[1] = -penetration * linearInertia[1] * inverseInertia;

    angularMove[0] = penetration * angularInertia[0] * inverseInertia;
    angularMove[1] = -penetration * angularInertia[1] * inverseInertia;

    // --- NOVO: Verificação de Limite de Rotação Excessiva ---
    const real angularLimit = 0.2f; // Constante de limite ajustável (ex: 0.2)

    for (unsigned i = 0; i < 2; i++)
    {
        if (body[i])
        {
            // Calcula o limite com base no tamanho do objeto (magnitude da posição relativa)
            real limit = angularLimit * relativeContactPosition[i].magnitude();

            if (real_abs(angularMove[i]) > limit)
            {
                real totalMove = linearMove[i] + angularMove[i];

                // Restringe o movimento angular ao limite máximo, mantendo o sinal
                if (angularMove[i] >= 0) {
                    angularMove[i] = limit;
                } else {
                    angularMove[i] = -limit;
                }

                // O movimento linear absorve o restante do deslocamento
                linearMove[i] = totalMove - angularMove[i];
            }
        }
    }
    // -------------------------------------------------------

    // 3. Aplica o movimento linear e angular aos corpos...
