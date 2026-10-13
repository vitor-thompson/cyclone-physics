#include <GL/glut.h>
#include <cyclone/particle.h>
#include "fireworks.cpp" // Inclui a lógica de fogos que criamos

// Array global de fogos de artifício
const unsigned int MAX_FIREWORKS = 100;
cyclone::Firework fireworks[MAX_FIREWORKS];

// Função chamada a cada quadro para atualizar a física
void updatePhysics()
{
    // Exemplo fixo de tempo por quadro (aprox. 60 FPS)
    cyclone::real duration = 0.016f; 

    for (unsigned int i = 0; i < MAX_FIREWORKS; i++) {
        if (fireworks[i].type > 0) {
            // Atualiza a física usando o seu motor!
            if (fireworks[i].update(duration)) {
                // Se o fogo explodiu, gerencia o payload (recolhe ou divide)
                fireworks[i].type = 0; 
            }
        }
    }
}

// Função chamada para desenhar os gráficos na tela
void renderScene()
{
    // Limpa a tela (fundo preto)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Configura o estilo para desenhar pontos (os fogos)
    glPointSize(4.0f);
    glBegin(GL_POINTS);

    for (unsigned int i = 0; i < MAX_FIREWORKS; i++) {
        if (fireworks[i].type > 0) {
            // Pega a posição exata calculada pelo seu motor de física!
            cyclone::Vector3 pos = fireworks[i].getPosition();
            
            // Desenha o ponto na coordenada (X, Y, Z)
            glVertex3f(pos.x, pos.y, pos.z);
        }
    }

    glEnd();
    
    // Troca os buffers para exibir a imagem na janela
    glutSwapBuffers();
}

// Função de controle de tempo do GLUT
void idle()
{
    updatePhysics();
    glutPostRedisplay(); // Pede para redesenhar a tela
}

int main(int argc, char** argv)
{
    // Inicializa a janela gráfica
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Motor Cyclone - Demo de Fogos de Artificio");

    // Registra as funções de callback do sistema gráfico
    glutDisplayFunc(renderScene);
    glutIdleFunc(idle);

    // Entra no loop principal da janela
    glutMainLoop();
    return 0;
}
