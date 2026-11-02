# Capitulo 9: Aplicacao de Corpo Rigido (A Rigid Body Application)

Este documento registra a construcao da primeira aplicacao pratica completa integrando a simulacao de corpos rigidos no motor **Cyclone**, correspondente ao **Capitulo 9** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 9 consolida a teoria dos Capitulos 7 e 8, aplicando a dinamica de corpos rigidos 3D, tensores de inercia, torques e integracao rotacional em uma demonstracao interativa de voo e pilotagem de uma aeronave.

## ✈️ 1. A Demonstracao de Voo (*The Ballistic / Flight Demo*)

Diferente das aplicacoes baseadas em particulas (onde a orientacao e ignorada), o simulador de voo exige o rastreamento preciso da orientacao e do tensor de inercia para simular as superfícies de controle aerodinamico: **lemes de direcao, profundores e ailerons**.

### Requisitos da Simulacao:

1. **Estado Tridimensional Completo:** Rastreamento continuo de posicao $\mathbf{p}$, orientacao $\mathbf{q}$, velocidade linear $\mathbf{v}$ e velocidade angular $\boldsymbol{\omega}$.

2. **Propulsao:** Aplicacao de forca de empuxo (*thrust*) gerada pelo motor da aeronave.

3. **Aerodinamica Integrada:** Geracao de sustentacao (*lift*) e arrasto (*drag*) dependendo da velocidade relativa e do angulo de ataque das asas.

## 🛩️ 2. Geracao de Forcas Aerodinamicas

Para modelar as asas e superfícies de controle sem a necessidade de simulação de fluidos complexa (CFD), o capitulo introduz o conceito de **Geradores de Forca Aerodinamica** (`Aero`).

### 2.1 Matriz de Superficie Aerodinamica

A forca resultante exercida pelo ar sobre uma asa e modelada por uma matriz $3 \times 3$ ($\mathbf{A}$) que relaciona o vetor de velocidade local do ar ($\mathbf{v}_{\text{local}}$) com a forca aerodinamica resultante ($\mathbf{F}_{\text{local}}$):

$$
\mathbf{F}_{\text{local}} = \mathbf{A} \cdot \mathbf{v}_{\text{local}}
$$

A matriz $\mathbf{A}$ codifica os coeficientes de sustentacao, atrito e arrasto lateral em relacao aos eixos locais da aeronave.

### 2.2 Forca Aerodinamica de Controle (`AeroControl`)

Para simular o movimento das superfícies de controle (como o movimento de um aileron ou profundor):

* A matriz aerodinamica e interpolada dinamicamente entre tres estados: posicao minima, posicao neutra e posicao maxima.

* A forca calculada no espaco local e aplicada em um ponto de ancoragem descentralizado $\mathbf{r}_{\text{asa}}$, gerando automaticamente tanto a forca linear de sustentacao/arrasto quanto o **torque rotacional** correspondente:

$$
\mathbf{F}_{\text{world}} = \mathbf{R} \cdot \mathbf{F}_{\text{local}}, \quad \boldsymbol{\tau}_{\text{world}} = (\mathbf{R} \cdot \mathbf{r}_{\text{asa}}) \times \mathbf{F}_{\text{world}}
$$

## 📐 3. Configuracao do Tensor de Inercia

Para a aeronave, o tensor de inercia e aproximado modelando a fuselagem como uma caixa orientada (*box tensor*) combinada com massas pontuais para as asas:

$$
\mathbf{I}_{\text{caixa}} = \begin{bmatrix} \frac{1}{12}m(h^2 + d^2) & 0 & 0 \\ 0 & \frac{1}{12}m(w^2 + d^2) & 0 \\ 0 & 0 & \frac{1}{12}m(w^2 + h^2) \end{bmatrix}
$$

Onde $w, h, d$ sao a largura, altura e profundidade da caixa envolvente da fuselagem.

## 💻 4. Estrutura da Implementacao (`aircraft.cpp`)

A aplicacao conecta os componentes desenvolvidos:

* **Corpo Rigido (`RigidBody`):** Representa a aeronave com sua massa, tensor de inercia e atributos de estado.

* **Geradores de Forca:** Instancias de `AeroControl` conectadas as asas esquerda/direita, estabilizador vertical e profundor horizontal.

* **Loop Principal:** A cada quadro, os comandos de entrada do usuario (teclado/joystick) alteram a deflexao das superfícies de controle, os geradores aplicam as forcas e torques no corpo rigido, e a chamada a `RigidBody::integrate()` atualiza a posicao e atitude da aeronave.