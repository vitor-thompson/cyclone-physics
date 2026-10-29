# Capítulo 14: Collision Resolution (Resolução de Colisões)

Este documento registra a implementação e os conceitos teóricos do **Capítulo 14** do livro *Game Physics Engine Development* (Ian Millington), focado na resolução de colisões para corpos rígidos em rotação no motor de física **Cyclone**.

---

## 1. Visão Geral do Pipeline de Resolução

A resolução de colisões para corpos rígidos rotativos exige o tratamento independente de dois problemas fundamentais:

1. **Resolução de Interpenetração (Posições):** Ajusta geometricamente as posições e orientações para retirar corpos rígidos de dentro uns dos outros de forma não linear.
2. **Resolução de Velocidade (Impulsos):** Modifica as velocidades lineares e angulares dos corpos com base no coeficiente de restituição e na conservação de momento.

```
       [ Detector de Colisões ]
                  │
                  ▼
         ( Array de Contatos )
                  │
                  ▼
       [ ContactResolver::resolveContacts ]
                  │
                  ├───> 1. prepareContacts()
                  ├───> 2. adjustPositions()  (Iterativo: Pior Penetração Primeiro)
                  └───> 3. adjustVelocities() (Iterativo: Maior Velocidade de Fechamento Primeiro)
```

---

## 2. Conceitos Matemáticos e Implementação

### 2.1 Base do Contato (`calculateContactBasis`)
Gera uma matriz de transformação $3 \times 3$ (`contactToWorld`) cuja primeira coluna é a normal do contato ($\mathbf{\hat{n}}$) e as outras duas colunas são eixos tangentes perpendiculares gerados via produto vetorial:

$$M_{\text{contactToWorld}} = \begin{bmatrix} \mathbf{\hat{n}} & \mathbf{t}_1 & \mathbf{t}_2 \end{bmatrix}$$

### 2.2 Resolução de Interpenetração (Projeção Não Linear)
A penetração é resolvida combinando translação linear e rotação angular. A inércia total na direção da normal é calculada combinando a massa inversa e o tensor de inércia inverso:

$$m^{-1}_{\text{eff}} = m^{-1} + \left( (I^{-1} (\mathbf{q}_{\text{rel}} \times \mathbf{\hat{n}})) \times \mathbf{q}_{\text{rel}} \right) \cdot \mathbf{\hat{n}}$$

* **Restrição de Rotação Excessiva:** Para evitar rotações feias ou infinitas quando o ponto de contato está muito afastado do centro de massa, limita-se a variação angular com base no tamanho do objeto:

```cpp
real limit = angularLimitConstant * relativeContactPosition.magnitude();
if (real_abs(angularMove) > limit) {
    // Restringe angularMove e transfere a diferença para o linearMove
}
```

### 2.3 Resolução de Velocidade (Impulsos Sem Atrito)
A variação da velocidade relativa de fechamento em coordenadas de contato é convertida em um vetor de impulso em coordenadas do mundo $\mathbf{g}_{\text{world}}$:

$$\mathbf{g} = M_{\text{contactToWorld}} \cdot \begin{bmatrix} \frac{\Delta v_{\text{desejado}}}{\text{deltaVelocity}} \\ 0 \\ 0 \end{bmatrix}$$

As atualizações no corpo rígido seguem:
* **Velocidade Linear:** $\Delta \mathbf{v} = m^{-1} \mathbf{g}$
* **Velocidade Angular:** $\Delta \boldsymbol{\omega} = I^{-1} (\mathbf{q}_{\text{rel}} \times \mathbf{\hat{n}})$

---

## 3. Algoritmo Iterativo de Resolução (*Best-First*)

Ambos os sub-resolvedores (`adjustPositions` e `adjustVelocities`) operam em loops iterativos de prioridade:

1. **Busca do Pior Caso:** Encontra o contato com o maior valor de sobreposição/fechamento.
2. **Resolução Isolada:** Aplica o ajuste apenas para aquele contato.
3. **Propagação Local (Atualização de Vizinhos):** Atualiza dinamicamente os valores de penetração/velocidade de outros contatos que compartilham os mesmos corpos rígidos, evitando ter que executar o detector de colisões novamente a cada iteração.

---

## 4. Estrutura do Código Modificado/Criado

### `include/cyclone/contacts.h`
* Classe `Contact`:
  * Membros de dados para normal, ponto de contato, restituição, profundidade de penetração e matrizes derivadas.
  * Métodos `calculateInternals`, `resolveVelocity`, `applyPositionChange` e `swapBodies`.
* Classe `ContactResolver`:
  * Controle do número máximo de iterações (`velocityIterations`, `positionIterations`) e limites de tolerância (`velocityEpsilon`, `positionEpsilon`).
  * Pipeline contendo `prepareContacts`, `adjustPositions`, `adjustVelocities` e `resolveContacts`.

### `src/contacts.cpp`
* Implementação completa da álgebra vetorial para o espaço de contato.
* Algoritmos iterativos com reajuste de contatos vizinhos.
* Tratamento de corpos estáticos e filtragem de velocidade para estabilização da gravidade em baixas velocidades.

---

## 5. Próximos Passos (Capítulo 15)

Com a resolução física básica completa, os próximos tópicos a serem implementados são:
1. **Fricção/Atrito (Friction):** Tratamento do impulso ao longo dos eixos tangentes $Y$ e $Z$.
2. **Contatos de Repouso (*Resting Contacts*):** Estabilização de pilhas e objetos apoiados no chão para eliminar vibrações (*jitter*).