# Capitulo 8: Leis do Movimento Rotacional (Laws of Motion for Rigid Bodies)

Este documento registra os conceitos teoricos e as formulacoes matematicas do movimento rotacional em corpos rigidos no motor **Cyclone**, correspondente ao **Capitulo 8** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 8 completa o modelo dinamico do corpo rigido, conectando **forcas aplicadas descentralizadamente** a **torques**, e o **tensor de inercia** a **aceleracao angular**.

## 🔄 1. Torque e Forcas Fora do Centro de Massa

Quando uma forca $\mathbf{F}$ e aplicada em um ponto $\mathbf{p}_r$ distante do centro de massa do objeto (vetor posicao relativa $\mathbf{r} = \mathbf{p}_r - \mathbf{p}_{\text{centro}}$), ela gera um movimento de rotacao conhecido como **Torque** ($\boldsymbol{\tau}$):

$$
\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F}
$$

* O vetor torque resultante e perpendicular ao plano formado pela forca e pelo vetor posicao de aplicacao.
* A magnitude do torque e proporcional a distancia do ponto de aplicacao e a componente perpendicular da forca.

## 🧮 2. O Tensor de Inercia

Assim como a massa $m$ mede a resistencia de um corpo a aceleracao linear, o **Tensor de Inercia** ($\mathbf{I}$) mede a resistencia de um corpo rigido a aceleracao angular em diferentes eixos de rotacao.

### 2.1 A Matriz de Inercia 3x3

Em tres dimensoes, o tensor de inercia e representado por uma matriz simetrica $3 \times 3$:

$$
\mathbf{I} = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{yx} & I_{yy} & I_{yz} \\ I_{zx} & I_{zy} & I_{zz} \end{bmatrix}
$$

* **Momentos de Inercia (Diagonal Principal):** Resistencia a rotacao em torno dos eixos locais $X$, $Y$ e $Z$.
* **Produtos de Inercia (Fora da Diagonal):** Medem a assimetria da distribuicao de massa. Alinhando os eixos locais com os eixos principais de inercia do objeto, os produtos fora da diagonal tornam-se zero.

### 2.2 Tensor de Inercia Inverso no Espaco Mundial

Para calcular a aceleracao angular no espaco do mundo a partir do torque aplicado, utiliza-se a matriz de inercia inversa no espaco mundial ($\mathbf{I}_{\text{world}}^{-1}$), transformada a partir da inercia local do corpo ($\mathbf{I}_{\text{body}}^{-1}$) e de sua matriz de orientacao ($\mathbf{R}$):

$$
\mathbf{I}_{\text{world}}^{-1} = \mathbf{R} \cdot \mathbf{I}_{\text{body}}^{-1} \cdot \mathbf{R}^T
$$

## ⚡ 3. Aceleracao Angular e Integracao Completa

A 2ª Lei de Newton para rotacoes estabelece a relacao entre torque acumulado e aceleracao angular ($\boldsymbol{\alpha}$):

$$
\boldsymbol{\tau} = \mathbf{I}_{\text{world}} \cdot \boldsymbol{\alpha} \implies \boldsymbol{\alpha} = \mathbf{I}_{\text{world}}^{-1} \cdot \boldsymbol{\tau}
$$

### Loop de Integracao do Corpo Rigido (`RigidBody::integrate`):

A cada passo de tempo $\Delta t$:

1. **Aceleracao Linear:** $\mathbf{a} = \mathbf{a}_{\text{constante}} + \mathbf{F}_{\text{acumulada}} \cdot w$
2. **Aceleracao Angular:** $\boldsymbol{\alpha} = \mathbf{I}_{\text{world}}^{-1} \cdot \boldsymbol{\tau}_{\text{acumulado}}$
3. **Atualizacao de Velocidade Linear:** $\mathbf{v}' = \mathbf{v} \cdot \gamma^{\Delta t} + \mathbf{a} \cdot \Delta t$
4. **Atualizacao de Velocidade Angular:** $\boldsymbol{\omega}' = \boldsymbol{\omega} \cdot \gamma_{\text{ang}}^{\Delta t} + \boldsymbol{\alpha} \cdot \Delta t$
5. **Atualizacao de Posicao:** $\mathbf{p}' = \mathbf{p} + \mathbf{v}' \cdot \Delta t$
6. **Atualizacao de Orientacao:** $\mathbf{q}' = \mathbf{q} + \left( \frac{1}{2} \boldsymbol{\omega}' \mathbf{q} \right) \Delta t$
7. **Normalizacao do Quaternion e Recalculo da Matriz Derivada.**