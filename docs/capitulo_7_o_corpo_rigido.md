# Capitulo 7: O Corpo Rigido (The Rigid Body)

Este documento registra a transicao da fisica de particulas para a simulacao de corpos rigidos tridimensionais no motor **Cyclone**, correspondente ao **Capitulo 7** do livro *Game Physics Engine Development* (Ian Millington).

Diferente das particulas pontuais, um **Corpo Rigido** possui extensao espacial, forma e massa distribuida. Isso introduz a dinamica rotacional e a necessidade de rastrear a orientacao do objeto no espaco 3D.

## 🔄 1. Orientacao e Espaco Rotacional

Para descrever completamente o estado de um corpo rigido no espaco 3D, sao necessarios dois conjuntos de coordenadas:

1. **Posicao Linear (**$\mathbf{p}$**):** Vetor 3D no espaco mundial (*world space*).

2. **Orientacao Rotacional (**$\mathbf{\Theta}$**):** Representa a rotacao do objeto em relacao a sua orientacao de referencia.

### 1.1 Quaternions para Orientacao

O motor utiliza **Quaternions unitarios** ($\mathbf{q} = [r, i, j, k]$) para armazenar a orientacao espacial de um corpo rigido.

* **Vantagens em relacao a Angulos de Euler:** Evita o problema do *Gimbal Lock* (trava de eixos) e permite interpolacao esferica suave (*Slerp*).

* **Vantagens em relacao a Matrizes de Rotacao:** Requer apenas 4 valores de ponto flutuante em vez de 9, facilitando a re-normalizacao para evitar erros de acumulo numerico.

### 1.2 Matriz de Transformacao (*Derived Data*)

Para transformar pontos do espaco local do objeto (*body space*) para o espaco do mundo (*world space*), uma matriz de transformacao $4 \times 4$ ($\mathbf{M}$) e derivada a partir da posicao e do quaternion de orientacao atualizados:

$$
\mathbf{M} = \begin{bmatrix} 
1 - 2j^2 - 2k^2 & 2ij - 2rk & 2ik + 2rj & p_x \\
2ij + 2rk & 1 - 2i^2 - 2k^2 & 2jk - 2ri & p_y \\
2ik - 2rj & 2jk + 2ri & 1 - 2i^2 - 2j^2 & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## 🌀 2. Velocidade Angular e Rotacao

A taxa de variacao da orientacao ao longo do tempo e descrita pela **velocidade angular** ($\boldsymbol{\omega}$), representada por um vetor 3D cuja direcao e o eixo de rotacao e cuja magnitude e a velocidade em radianos por segundo.

### Atualizacao de Quaternions por Velocidade Angular:

A derivada de um quaternion de orientacao $\mathbf{q}$ em relacao ao tempo e dada por:

$$
\dot{\mathbf{q}} = \frac{1}{2} \boldsymbol{\omega} \mathbf{q}
$$

No passo de integracao numerica com intervalo $\Delta t$:

$$
\mathbf{q}' = \mathbf{q} + \left( \frac{1}{2} \boldsymbol{\omega} \mathbf{q} \right) \Delta t
$$

Apos a atualizacao, $\mathbf{q}'$ e normalizado para garantir que permaneca como um quaternion unitario valido.

## 💻 3. A Estrutura da Classe `RigidBody`

A classe `RigidBody` (definida em `include/cyclone/body.h`) expande a estrutura base de estado do motor:

### Atributos Principais:

* `position` (`Vector3`): Posicao linear no mundo.
* `orientation` (`Quaternion`): Orientacao rotacional.
* `velocity` (`Vector3`): Velocidade linear.
* `rotation` (`Vector3`): Velocidade angular ($\boldsymbol{\omega}$).
* `transformMatrix` (`Matrix4`): Matriz derivada para transformacao local-para-mundo.
* `inverseMass` (`real`): Massa inversa ($1/m$).
* `linearDamping` / `angularDamping` (`real`): Coeficientes de amortecimento para estabilidade numerica.

### Integracao Basica (`integrate`):

Semelhante a particula, mas atualiza posicao linear e orientacao rotacional simultaneamente com base nas velocidades correspondentes.