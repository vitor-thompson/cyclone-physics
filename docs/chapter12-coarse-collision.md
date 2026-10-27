# Capítulo 12: Detecção Grossa de Colisões (Coarse Collision Detection)

Este capítulo introduz a primeira fase do pipeline de detecção de colisões do motor **Cyclone**: a **Detecção Grossa (*Coarse Collision Detection*)**. O objetivo desta etapa é filtrar rapidamente os pares de objetos que estão próximos o suficiente para colidir, descartando a grande maioria do cenário e evitando checagens geométricas detalhadas desnecessárias.

---

## 1. O Pipeline de Colisão

Para manter o desempenho em tempo real, a detecção de colisões é dividida em duas etapas:
1. **Detecção Grossa (Coarse):** Um filtro rápido e conservador baseado em volumes simples (esferas/caixas) e estruturas espaciais para gerar uma lista de potenciais contatos. Nunca deve gerar falsos negativos (perder uma batida real).
2. **Detecção Fina (Fine):** Uma análise geométrica detalhada apenas sobre os candidatos selecionados na fase anterior, calculando o ponto exato de contato, a normal e a profundidade de penetração (Capítulo 13).

---

## 2. Volumes Envolventes e Esferas (`BoundingSphere`)

Para simplificar os testes iniciais, os objetos do motor são envolvidos por **Esferas Envolventes (`BoundingSphere`)**. 
* **Vantagem:** A verificação de sobreposição entre esferas é extremamente rápida, utilizando a distância ao quadrado para evitar o cálculo custoso de raízes quadradas.

```cpp
bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
    real distanceSquared = (center - other->center).squareMagnitude();
    real radiusSum = radius + other->radius;
    return distanceSquared < (radiusSum * radiusSum);
}
