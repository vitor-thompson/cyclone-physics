# Capítulo 15: Contatos de Repouso e Fricção (Resting Contacts & Friction)

Este documento registra os estudos, a fundamentação matemática e a implementação do **Capítulo 15** do livro *Game Physics Engine Development* (Ian Millington) no motor de física **Cyclone**.

---

## 1. Visão Geral

O Capítulo 15 aborda a transição de impactos colisionais instantâneos para **contatos de repouso** (objetos empilhados ou deslizando em superfícies) e a introdução da **fricção isotrópica**.

No motor *Cyclone*, contatos de repouso são simulados como **micro-colisões**, utilizando impulsos em vez de forças de reação contínuas globais.

---

## 2. Conceitos-Chave e Fundamentação Matemática

### 2.1 Eliminação de Vibrações (*Anti-Jitter*)
Para evitar que objetos em repouso quiquem infinitamente devido à aceleração acumulada da gravidade:
1. **Compensação de Aceleração:** A velocidade induzida pela aceleração do último quadro ($v_{\text{acc}}$) é subtraída antes do cálculo de restituição.
2. **Corte de Restituição (`velocityLimit`):** Se a magnitude da velocidade no ponto de contato for inferior a um limite mínimo ($\approx 0.1 \text{ m/s}$), o coeficiente de restituição $e$ é zerado ($e = 0.0$).

$$\Delta v_{\text{desejado}} = -v_{\text{contato}} - e \cdot (v_{\text{contato}} - v_{\text{acc}})$$

### 2.2 Fricção Isotrópica baseada em Impulsos
O atrito é modelado convertendo a força de reação no impulso normal $g_x$. O limite de atrito estático é estabelecido por:

$$|g_{\text{planar}}| \le \mu \cdot g_x$$

Onde $g_{\text{planar}} = \sqrt{g_y^2 + g_z^2}$.

- **Atrito Estático:** Se $|g_{\text{planar}}| \le \mu \cdot g_x$, o impulso cancela totalmente o deslizamento tangencial ($y$ e $z$).
- **Atrito Dinâmico:** Se o impulso necessário superar o limite, o vetor de impulso tangencial é escalado mantendo sua direção, mas com magnitude limitada a $\mu \cdot g_x$.

### 2.3 Matrizes Antissimétricas (*Skew-Symmetric Matrices*)
Para resolver a variação de velocidade 3D gerada por rotações induzidas por impulsos tangenciais, utiliza-se a forma matricial do produto vetorial $a \times b = A_s b$:

$$A_s = \begin{bmatrix} 0 & -a_z & a_y \\ a_z & 0 & -a_x \\ -a_y & a_x & 0 \end{bmatrix}$$

---

## 3. Alterações na Estrutura do Código

### `include/cyclone/core.h`
- Adicionado o método `setSkewSymmetric(const Vector3 vector)` na classe `Matrix3`.

### `include/cyclone/bodies.h` & `src/bodies.cpp`
- Suporte a rastreamento de aceleração do quadro anterior (`lastFrameAcceleration`).

### `src/contacts.cpp`
- Atualização do método `calculateDesiredDeltaVelocity` para compensar gravidade e cortar restituição em baixas velocidades.
- Reformulação do método `resolveVelocity` para usar a matriz 3x3 de variação de velocidade (`deltaVelocity`), inverter a matriz para encontrar o impulso tridimensional e aplicar a transição entre atrito estático e dinâmico.

---

## 4. Limitações da Resolução Sequencial

Como os contatos são resolvidos **sequencialmente** e não de forma simultânea, pilhas muito altas de objetos podem apresentar um leve deslizamento lateral ao longo do tempo. Esse comportamento é atenuado na prática através do **sistema de sono (*sleeping*)**, abordado no Capítulo 16.