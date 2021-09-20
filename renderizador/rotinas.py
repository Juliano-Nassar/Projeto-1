#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Rotinas de operação de nós X3D.

Desenvolvido por:
Disciplina: Computação Gráfica
Data:
"""

import gpu          # Simula os recursos de uma GPU
import numpy as np
from auxiliares import Area
from auxiliares import TaDentro

#################################################################################
# NÃO USAR MAIS ESSE ARQUIVO. AS ROTINAS DEVEM SER IMPLEMENTADAS AGORA NO gl.GL #
#################################################################################

# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#Polypoint2D
def polypoint2D(point, colors):
    """Função usada para renderizar Polypoint2D."""
    # Nessa função você receberá pontos no parâmetro point, esses pontos são uma lista
    # de pontos x, y sempre na ordem. Assim point[0] é o valor da coordenada x do
    # primeiro ponto, point[1] o valor y do primeiro ponto. Já point[2] é a
    # coordenada x do segundo ponto e assim por diante. Assuma a quantidade de pontos
    # pelo tamanho da lista e assuma que sempre vira uma quantidade par de valores.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o Polypoint2D
    # você pode assumir o desenho dos pontos com a cor emissiva (emissiveColor).
    
    # Transforma as cores normalizadas (0-1) para janela rgb (0-255)
    r = round((colors['emissiveColor'][0])*255)
    g = round((colors['emissiveColor'][1])*255)
    b = round((colors['emissiveColor'][2])*255)
    
    # Para cada coordenada desenha um ponto
    for i in range(0,len(point),2):
        
        # Sempre arredonda pra baixo (Para pegar o centro de pixel mais próximo do ponto)
        xi = int(point[i])
        yi = int(point[i+1])
        gpu.GPU.set_pixel(xi, yi, r, g, b) # altera um pixel da imagem (u, v, r, g, b)

# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#Polyline2D
def polyline2D(lineSegments, colors):
    """Função usada para renderizar Polyline2D."""
    # Nessa função você receberá os pontos de uma linha no parâmetro lineSegments, esses
    # pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o valor da
    # coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto. Já point[2] é
    # a coordenada x do segundo ponto e assim por diante. Assuma a quantidade de pontos
    # pelo tamanho da lista. A quantidade mínima de pontos são 2 (4 valores), porém a
    # função pode receber mais pontos para desenhar vários segmentos. Assuma que sempre
    # vira uma quantidade par de valores.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o Polyline2D
    # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
    
    # Transforma as cores normalizadas (0-1) para janela rgb (0-255)
    r = round((colors['emissiveColor'][0])*255)
    g = round((colors['emissiveColor'][1])*255)
    b = round((colors['emissiveColor'][2])*255)
    
    # Desenha uma linha entre cada par de pontos
    for i in range(0,len(lineSegments)-2,2):
        
        x0 = lineSegments[i]
        y0 = lineSegments[i + 1]
        
        x1 = lineSegments[i + 2]
        y1 = lineSegments[i + 3]
        
        # Se a linha é vertical, pintar iterando entre os ys (Coeficiente angular é infinito)
        if int(x0) == int(x1):
            
            y0 = int(y0)
            y1 = int(y1)
            xref = int(x0)
            
            step = 1
            
            if y0>y1:
                step = -step
            
            for y in range(y0,y1+step,step):
                gpu.GPU.set_pixel(xref, int(y), r, g, b) # altera um pixel da imagem (u, v, r, g, b)
            
        else:  
            # Equação da reta
            # y = ax + b
            a = (y1-y0)/(x1-x0)
            b = y1 - a*x1
            
            # Passo relativo a inclinação da reta
            if a == 0: # Se a linha for horizonal, usa passo de 1 unidade
                step = 1
            else:
                step = 1/abs(a)
                
                # Caso a inclinação seja muito pequena, utiliza 1 unidade como passo
                if step>1:
                    step = 1
                    
            # Caso x inicial seja maior que o x final, utiliza um passo negatico (limitação do np.arange)
            if x0>x1:
                step = -step
            
            # A cada passo entre o ponto p0 e p1, calcula o valor do ponto, arredonda pra baixo e pinta
            for x in np.arange(x0,x1+step/2,step):
                y = a*x + b
                gpu.GPU.set_pixel(int(x), int(y), r, g, b) # altera um pixel da imagem (u, v, r, g, b)
                
# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#TriangleSet2D
def triangleSet2D(vertices, colors):
    """Função usada para renderizar TriangleSet2D."""
    # Nessa função você receberá os vertices de um triângulo no parâmetro vertices,
    # esses pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o
    # valor da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto.
    # Já point[2] é a coordenada x do segundo ponto e assim por diante. Assuma que a
    # quantidade de pontos é sempre multiplo de 3, ou seja, 6 valores ou 12 valores, etc.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet2D
    # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
    r = round((colors['emissiveColor'][0])*255)
    g = round((colors['emissiveColor'][1])*255)
    b = round((colors['emissiveColor'][2])*255)
    # Desenha uma linha entre cada par de pontos
    for i in range(0,len(vertices),6):
        x1 = vertices[i]
        y1 = vertices[i+1]
        x2 = vertices[i+2]
        y2 = vertices[i+3]
        x3 = vertices[i+4]
        y3 = vertices[i+5]
        pixelList = [vertices[i],vertices[i+1],vertices[i+2],vertices[i+3],vertices[i+4],vertices[i+5]]

        # Encontra limites do triângulo, assim como pixel inicial
        starter = [int(x1),int(y1)]
        lowerX = int(x1)
        higherX = int(x1)
        lowerY = int(y1)
        for j in range(2,5,2) :
            x = int(pixelList[j])
            y = int(pixelList[j+1])
            if x > higherX:
                higherX = int(x)
            if x < lowerX:
                lowerX = int(x)
            if y > starter[1]:
                starter = [int(x),int(y)]
            if y < lowerY:
                lowerY = int(y)

        #encontra o primeiro pixel a ser pintado
        notDone = True
        while notDone:
            if starter[1] < lowerY:
                notDone = True
                break
                
            color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starter[0], starter[1])
            if color_lv > 0:
                gpu.GPU.set_pixel(starter[0], starter[1], int(r*color_lv), int(g*color_lv), int(b*color_lv))

            starterX = starter[0] -1
            while starterX >= lowerX:        
                color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starterX, starter[1])            
                if color_lv > 0:
                    gpu.GPU.set_pixel(starterX, starter[1], int(r*color_lv), int(g*color_lv), int(b*color_lv))
                starterX -= 1


            while starterX <= higherX:
                color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starterX, starter[1])            
                if color_lv > 0:
                    gpu.GPU.set_pixel(starterX, starter[1], int(r*color_lv), int(g*color_lv), int(b*color_lv))
                starterX += 1

            starter[1] = starter[1]-1
