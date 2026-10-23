# Capítulo 9: Matemática de Rotações (Motor Cyclone)

Esta documentação resume os principais conceitos matemáticos e classes implementadas no Capítulo 9 do motor de física *Cyclone* (baseado no livro *Game Physics Engine Development* de Ian Millington).

---

## 1. Visão Geral dos Conceitos

* **Convenção:** O motor utiliza multiplicação à direita e vetores coluna ($v = Mv$).
* **Posições vs. Direções:** Posições sofrem rotação e translação. Vetores de direção devem ser **imunes à translação**, exigindo transformações especializadas para ignorar deslocamentos espaciais.
* **Quatérnions ($Quaternion$):** Utilizados para gerenciar orientações em 3 graus de liberdade (3DOF), evitando o problema de travamento de eixos (*Gimbal Lock*).

---

## 2. Resumo das Classes e Funções Principais

### **Vector3 (Fundação Vetorial)**
Gerencia coordenadas espaciais 3D e operações fundamentais.
* **Normalização (`normalize()`):** Converte o vetor em um vetor unitário.
* **Produto Escalar e Vetorial:** Essenciais para cálculos de torques e alinhamentos.
* **Atualização Otimizada (`addScaledVector()`):** Atualiza posições somando velocidade vezes o tempo ($p + v \cdot dt$) sem alocar memória temporária.

### **Quaternion (Orientação e Rotação)**
Armazenado através de uma `union` contendo componentes nomeados (`r, i, j, k`) e um array (`data[4]`).
* **Normalização (`normalize()`):** Corrige desvios numéricos acumulados para manter a magnitude em $1.0$.
* **Combinação (`operator*=`):** Multiplica dois quatérnions para acumular rotações consecutivas ($q \times p$).
* **Rotação por Vetor (`rotateByVector()`):** Atualiza a orientação usando um vetor de velocidade angular escalado pelo tempo.

### **Matrix3 (Rotações e Inércia - 3x3)**
* **Orientação (`setOrientation()`):** Converte um quatérnion de orientação diretamente em uma matriz de rotação 3x3.
* **Inversão e Transposta:** Permitem reverter transformações ou transpor matrizes de rotação (onde a inversa é igual à transposta).

### **Matrix4 (Transformações Homogêneas - 3x4)**
Combina uma matriz de rotação 3x3 com um vetor de posição de 3 elementos.
* **Posição e Orientação (`setOrientationAndPos()`):** Configura a matriz completa combinando o quatérnion e a posição cartesiana.
* **Transformações Especializadas:**
  * `transformInverse()`: Converte coordenadas do mundo para o espaço local de forma otimizada.
  * `transformDirection()` e `transformInverseDirection()`: Transformam direções ignorando a translação.

---

## 3. Exemplo Prático de Uso

```cpp
using namespace cyclone;

// Definindo posição e orientação de um corpo rígido
Vector3 position(0.0f, 10.0f, 0.0f);
Quaternion orientation(1.0f, 0.0f, 0.0f, 0.0f); // Sem rotação

Matrix4 transformMatrix;
transformMatrix.setOrientationAndPos(orientation, position);

// Transformando um ponto local do objeto para o espaço global (World)
Vector3 localPoint(0.0f, 1.0f, 0.0f);
Vector3 worldPoint = transformMatrix * localPoint;
