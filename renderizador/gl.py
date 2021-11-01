#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Biblioteca Gráfica / Graphics Library.

Desenvolvido por: <SEU NOME AQUI>
Disciplina: Computação Gráfica
Data:
"""

import gpu          # Simula os recursos de uma GPU
import numpy as np
from methods import Quaternio, Vector
from auxiliares import *
import time
import math

class GL:
    """Classe que representa a biblioteca gráfica (Graphics Library)."""

    width = 800   # largura da tela
    height = 600  # altura da tela
    near = 0.01   # plano de corte próximo
    far = 1000    # plano de corte distante


    def draw_pixel(point, color):
        # Z-BUFFER
        if point[2] <= gpu.GPU.read_pixels([point[0], point[1]],gpu.GPU.DEPTH_COMPONENT16):
            gpu.GPU.draw_pixels([point[0], point[1]], gpu.GPU.RGB8, [int(color[0]), int(color[1]), int(color[2])])
            gpu.GPU.draw_pixels([point[0], point[1]], gpu.GPU.DEPTH_COMPONENT16, point[2])
            

    def triangleSet2D(vertices, colors, z_list = [1, 1, 1]):
        """Função usada para renderizar TriangleSet2D."""
        # Nessa função você receberá os vertices de um triângulo no parâmetro vertices,
        # esses pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o
        # valor da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto.
        # Já point[2] é a coordenada x do segundo ponto e assim por diante. Assuma que a
        # quantidade de pontos é sempre multiplo de 3, ou seja, 6 valores ou 12 valores, etc.
        # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet2D
        # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).


        if 'diffuseColor' in colors:
            r = round((colors['diffuseColor'][0])*255)
            g = round((colors['diffuseColor'][1])*255)
            b = round((colors['diffuseColor'][2])*255)

            color = [r,g,b]
            Oa = 0#colors['ambientIntensity']
            Odrgb = np.array(colors['diffuseColor'])
            Osrgb = np.array(colors['specularColor'])
            Oergb = np.array(colors['emissiveColor'])
            shininess = colors['shininess']

            if GL.is_directional_light:
                Ilrgb = GL.directional_light['Ilrgb']
                Ii = GL.directional_light['Ii']
                Iia = GL.directional_light['Iia']
                l = GL.directional_light['L']
                r = 1
            
            condition = 'single_color'
        else:
            c1 = colors[:3]
            c2 = colors[3:6]
            c3 = colors[6:]
            condition = 'multi_color'
        # Desenha uma linha entre cada par de pontos
        for i in range(0,len(vertices),6):
            x1 = vertices[i]
            y1 = vertices[i+1]
            z1 = z_list[i]
            
            x2 = vertices[i+2]
            y2 = vertices[i+3]
            z2 = z_list[i+1]
            
            x3 = vertices[i+4]
            y3 = vertices[i+5]
            z3 = z_list[i+2]
            
            p0 = np.array([x3,y3,z3])
            p1 = np.array([x2,y2,z2])
            p2 = np.array([x1,y1,z1])
            

            n = calc_normal(p0,p1,p2)
            a,b,c = plane_eq(x1, y1, z1, x2, y2, z2, x3, y3, z3)
            
            pixelList = [vertices[i],vertices[i+1],vertices[i+2],vertices[i+3],vertices[i+4],vertices[i+5]]
            
            # TRANSFORMAR PIXEL SEARCH EM UMA FUNÇÃO
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
            state = "first"
            while notDone:
                # Starter[0] = x do pixel novo
                # Starter[1] = y do pixel novo
                # Maquina de estados
                if state == "first":
                    pixelX = starter[0]
                    pixelY = starter[1]
                    state = "left"

                elif state == "neutral":
                    pixelX = starter[0]
                    pixelY = pixelY - 1
                    state = "left"
                    if pixelY < lowerY:
                        break

                elif state == "left":
                    pixelX = pixelX - 1
                    if lowerX > pixelX:
                        pixelX = starter[0]
                        state = "right"
                        continue
                
                elif state == "right":
                    pixelX = pixelX + 1
                    if higherX < pixelX:
                        state = "neutral"
                        continue

                # Loop principal
                if TaDentro(x1, y1, x2, y2, x3, y3, pixelX, pixelY):

                    # Baricentro
                    alpha,beta,gama,Z = Baricentro(x1, y1, z1, x2, y2, z2, x3, y3, z3, pixelX, pixelY)

                    
                    # Interpolar com cor perspectiva
                    if condition == "multi_color":
                        color = QueCorDeus(alpha,c1,z1,beta,c2,z2,gama,c3,z3,Z)
                    
                    

                    z = calc_z(pixelX,pixelY,a,b,c)
                    point = [pixelX, pixelY, z]

                    if GL.is_directional_light or GL.is_point_light:
                        v = -np.array(point)
                        v = v/np.linalg.norm(v)

                        color = calc_light(Oa, Odrgb, Osrgb, Oergb, Ilrgb, Ii, Iia, r, n, l, v, shininess)
                        color = color*255
                        color = color.astype(np.uint8)
                    GL.draw_pixel(point,color)
            
    def triangleSet2D_texture(vertices, texture, texture_coordinates , z_list = [0, 0, 0]):
        """Função usada para renderizar TriangleSet2D."""
        # Nessa função você receberá os vertices de um triângulo no parâmetro vertices,
        # esses pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o
        # valor da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto.
        # Já point[2] é a coordenada x do segundo ponto e assim por diante. Assuma que a
        # quantidade de pontos é sempre multiplo de 3, ou seja, 6 valores ou 12 valores, etc.
        # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet2D
        # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
        
        # Desenha uma linha entre cada par de pontos
        
        tex_res,_,_ = texture.shape
        tex_res = tex_res-1
        
        for i in range(0,len(vertices),6):
            x1 = vertices[i]
            y1 = vertices[i+1]
            z1 = z_list[i]
            
            x2 = vertices[i+2]
            y2 = vertices[i+3]
            z2 = z_list[i+1]
            
            x3 = vertices[i+4]
            y3 = vertices[i+5]
            z3 = z_list[i+2]
            
            a,b,c = plane_eq(x1, y1, z1, x2, y2, z2, x3, y3, z3)
            
            # text coords
            u0 = texture_coordinates[i]
            v0 = texture_coordinates[i+1]
            
            u1 = texture_coordinates[i+2]
            v1 = texture_coordinates[i+3]     
            
            u2 = texture_coordinates[i+4]
            v2 = texture_coordinates[i+5]
            
            pixelList = [vertices[i],vertices[i+1],vertices[i+2],vertices[i+3],vertices[i+4],vertices[i+5]]
            
            # TRANSFORMAR PIXEL SEARCH EM UMA FUNÇÃO
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
                    #gpu.GPU.draw_pixels([starter[0], starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])
                    
                    # Baricentro
                    alpha,beta,gama,_ = Baricentro(x1, y1, z1, x2, y2, z2, x3, y3, z3, starter[0], starter[1])
                    
                    U = (u0*alpha + u1*beta + u2*gama)*tex_res
                    V = (v0*alpha + v1*beta + v2*gama)*tex_res
                    
                    U = round(U)
                    V = round(V)
                    
                    color = texture[U,V][:3]
                    
                    
                    z = calc_z(starter[0],starter[1],a,b,c)
                    
                    point = [starter[0], starter[1], z]
                    
                    GL.draw_pixel(point,color)
                    
                starterX = starter[0] -1
                while starterX >= lowerX:        
                    color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starterX, starter[1])            
                    if color_lv > 0:
                        #gpu.GPU.draw_pixels([starterX, starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])
                        # Baricentro
                        alpha,beta,gama,Z = Baricentro(x1, y1, z1, x2, y2, z2, x3, y3, z3, starterX, starter[1])
                        
                        U = (u0*alpha + u1*beta + u2*gama)*tex_res
                        V = (v0*alpha + v1*beta + v2*gama)*tex_res
                        
                        U = round(U)
                        V = round(V)
                        
                        color = texture[U,V][:3] 
                        
                        z = calc_z(starterX,starter[1],a,b,c)
                        point = [starterX, starter[1], z ]
                        GL.draw_pixel(point,color)
                    starterX -= 1


                while starterX <= higherX:
                    color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starterX, starter[1])            
                    if color_lv > 0:
                        #gpu.GPU.draw_pixels([starterX, starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])
                        # Baricentro
                        alpha,beta,gama,Z = Baricentro(x1, y1, z1, x2, y2, z2, x3, y3, z3, starterX, starter[1])
                        
                        U = (u0*alpha + u1*beta + u2*gama)*tex_res
                        V = (v0*alpha + v1*beta + v2*gama)*tex_res
                        
                        U = round(U)
                        V = round(V)
                        
                        color = texture[U,V][:3] 

                        z = calc_z(starterX,starter[1],a,b,c)
                        point = [starterX, starter[1], z]
                        
                        GL.draw_pixel(point,color)
                        
                    
                    starterX += 1

                starter[1] = starter[1]-1
            # TRANSFORMAR PIXEL SEARCH EM UMA FUNÇÃO FIM            
            
    @staticmethod
    def setup(width, height, near=0.01, far=1000):
        """Definr parametros para câmera de razão de aspecto, plano próximo e distante."""
        GL.width = width
        GL.height = height
        GL.ascpect = width/height
        GL.near = near
        GL.far = far
        
        GL.is_directional_light = False
        GL.is_point_light = False
        
        M_I = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        GL.stack = [M_I]
        
        GL.box_triangles = [1,2,3,
                            2,3,4,
                            2,4,6,
                            4,6,8,
                            3,4,7,
                            4,7,8,
                            1,2,6,
                            1,5,6,
                            6,7,8,
                            5,6,7,
                            3,5,7,
                            1,3,5]
                            
    @staticmethod
    def triangleSet(point, colors, is_texture = False, texture_coordinates=[]):
        """Função usada para renderizar TriangleSet."""
        # Nessa função você receberá pontos no parâmetro point, esses pontos são uma lista
        # de pontos x, y, e z sempre na ordem. Assim point[0] é o valor da coordenada x do
        # primeiro ponto, point[1] o valor y do primeiro ponto, point[2] o valor z da
        # coordenada z do primeiro ponto. Já point[3] é a coordenada x do segundo ponto e
        # assim por diante.
        # No TriangleSet os triângulos são informados individualmente, assim os três
        # primeiros pontos definem um triângulo, os três próximos pontos definem um novo
        # triângulo, e assim por diante.
        # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet
        # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
        
        condition = 'multi_color'
        if is_texture:
            condition = 'is_texture'
        elif 'diffuseColor' in colors:
            condition = 'single_color'

        aux = 1
        for i in range(0,len(point)-8,9):
            tri = []
            tri.append([point[i],point[i+3],point[i+6]])
            tri.append([point[i+1],point[i+4],point[i+7]])
            tri.append([point[i+2],point[i+5],point[i+8]])
            tri.append([1,1,1])
            tri_M = np.array(tri)
            
            # Transformações objeto-mundo
            tri_M = np.matmul(GL.stack[-1],tri_M)
            
            # Transformações Mundo-camera
            tri_M = np.matmul(GL.Look_At,tri_M)
            z_list = tri_M[-2]
            
            # Transformações perspectiva
            tri_M = np.matmul(GL.camera_M_P,tri_M)
            
            # Transformações NDC
            tri_M = tri_M/tri_M[-1]
            
            # Transformaões mapping tela
            tri_M = np.matmul(GL.tela_M,tri_M)
            
            # Lista de pontos x,y
            tri_M = tri_M[:2,:].T.flatten().tolist()
            
            if condition == 'is_texture':
                text_index = int(i*6/9)
                tex_coordinates = texture_coordinates[text_index:text_index+6]
                GL.triangleSet2D_texture(tri_M,colors,z_list = z_list, texture_coordinates = tex_coordinates)
                
            elif condition == 'multi_color':
                GL.triangleSet2D(tri_M,colors[i:i+9],z_list = z_list)
                
            else:
                GL.triangleSet2D(tri_M,colors,z_list = z_list)
                
            aux+=1

    @staticmethod
    def viewpoint(position, orientation, fieldOfView):

        """Função usada para renderizar (na verdade coletar os dados) de Viewpoint."""
        # Na função de viewpoint você receberá a posição, orientação e campo de visão da
        # câmera virtual. Use esses dados para poder calcular e criar a matriz de projeção
        # perspectiva para poder aplicar nos pontos dos objetos geométricos.

        # O print abaixo é só par"a vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        # Matriz para transformar a posição dos objetos do mundo para a posição em relação a câmera
        # O inverso de um rotação é a rotação para o lado contrário
        q = Quaternio(angle = -orientation[-1], axis = orientation[:3])
        GL.camera_M_R = q.rotation_matrix().v

        # O inverso de uma translação é uma translação com sinais trocados em x,y e z
        GL.camera_M_T = np.array([[1,0,0,-position[0]],
                                   [0,1,0,-position[1]],
                                   [0,0,1,-position[2]],
                                   [0,0,0,      1   ]])
                                   
        GL.Look_At = np.matmul(GL.camera_M_R,GL.camera_M_T)
        
        # FOVy e matriz de perspectiva
        FOVd = fieldOfView
        GL.FOVd = FOVd
        GL.FOVy = 2*np.arctan(np.tan(FOVd/2)*(GL.height/(np.sqrt(GL.height**2+GL.width**2))))
        
        
        GL.top = GL.near*np.tan(GL.FOVy)
        GL.bottom = -GL.top
        GL.right = GL.top*GL.ascpect
        GL.left = -GL.right
        
        # Matriz perspectiva
        GL.camera_M_P = np.array([[GL.near/GL.right,0,0,0],
                                   [0,GL.near/GL.top,0,0],
                                   [0,0,-(GL.far+GL.near)/(GL.far-GL.near),-2*GL.far*GL.near/(GL.far-GL.near)],
                                   [0,0,-1,0]])
        
        GL.tela_M = np.array([[GL.width/2,0,0,GL.width/2],
                              [0,-GL.height/2,0,GL.height/2],
                              [0,0,1,0],
                              [0,0,0,1]])
        
        
    @staticmethod
    def transform_in(translation, scale, rotation):
        """Função usada para renderizar (na verdade coletar os dados) de Transform."""
        # A função transform_in será chamada quando se entrar em um nó X3D do tipo Transform
        # do grafo de cena. Os valores passados são a escala em um vetor [x, y, z]
        # indicando a escala em cada direção, a translação [x, y, z] nas respectivas
        # coordenadas e finalmente a rotação por [x, y, z, t] sendo definida pela rotação
        # do objeto ao redor do eixo x, y, z por t radianos, seguindo a regra da mão direita.
        # Quando se entrar em um nó transform se deverá salvar a matriz de transformação dos
        # modelos do mundo em alguma estrutura de pilha.
        M_I = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        
        if scale:
            M_S = np.array([[scale[0],0,0,0],
                                [0,scale[1],0,0],
                                [0,0,scale[2],0],
                                [0,0,   0,    1]])
            M_I = np.matmul(M_S,M_I)
            
        if rotation:
            q = Quaternio(angle = rotation[-1], axis = rotation[:3])
            M_R = q.rotation_matrix().v
            M_I = np.matmul(M_R,M_I)
            
        if translation:
            M_T = np.array([[1,0,0,translation[0]],
                                [0,1,0,translation[1]],
                                [0,0,1,translation[2]],
                                [0,0,0,      1      ]])
            
            M_I = np.matmul(M_T,M_I)
        
        M_final = np.matmul(GL.stack[-1],M_I)
        GL.stack.append(M_final)
    @staticmethod
    def transform_out():
        """Função usada para renderizar (na verdade coletar os dados) de Transform."""
        # A função transform_out será chamada quando se sair em um nó X3D do tipo Transform do
        # grafo de cena. Não são passados valores, porém quando se sai de um nó transform se
        # deverá recuperar a matriz de transformação dos modelos do mundo da estrutura de
        # pilha implementada.
        GL.stack.pop()
        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        #print("Saindo de Transform")

    @staticmethod
    def triangleStripSet(point, stripCount, colors):
        """Função usada para renderizar TriangleStripSet."""
        # A função triangleStripSet é usada para desenhar tiras de triângulos interconectados,
        # você receberá as coordenadas dos pontos no parâmetro point, esses pontos são uma
        # lista de pontos x, y, e z sempre na ordem. Assim point[0] é o valor da coordenada x
        # do primeiro ponto, point[1] o valor y do primeiro ponto, point[2] o valor z da
        # coordenada z do primeiro ponto. Já point[3] é a coordenada x do segundo ponto e assim
        # por diante. No TriangleStripSet a quantidade de vértices a serem usados é informado
        # em uma lista chamada stripCount (perceba que é uma lista). Ligue os vértices na ordem,
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. Cuidado com a orientação dos vértices, ou seja,
        # todos no sentido horário ou todos no sentido anti-horário, conforme especificado.

        for i in range(6,len(point)-2,3):
            tri = []
            tri.append([point[i],point[i-3],point[i-6]])
            tri.append([point[i+1],point[i-2],point[i-5]])
            tri.append([point[i+2],point[i-1],point[i-4]])
            tri.append([1,1,1])
            tri_M = np.array(tri)

            tri_M = np.matmul(GL.stack[-1],tri_M)
            
            # Transformações Mundo-camera
            tri_M = np.matmul(GL.Look_At,tri_M)
            
            # Transformações perspectiva
            tri_M = np.matmul(GL.camera_M_P,tri_M)
            
            # Transformações NDC
            tri_M = tri_M/tri_M[-1]
            
            # Transformaões mapping tela
            tri_M = np.matmul(GL.tela_M,tri_M)
            
            # Lista de pontos x,y
            tri_M = tri_M[:2,:].T.flatten().tolist()
            
            
            GL.triangleSet2D(tri_M,colors)



    @staticmethod
    def indexedTriangleStripSet(point, index, colors, inverse = False):
        """Função usada para renderizar IndexedTriangleStripSet."""
        # A função indexedTriangleStripSet é usada para desenhar tiras de triângulos
        # interconectados, você receberá as coordenadas dos pontos no parâmetro point, esses
        # pontos são uma lista de pontos x, y, e z sempre na ordem. Assim point[0] é o valor
        # da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto, point[2]
        # o valor z da coordenada z do primeiro ponto. Já point[3] é a coordenada x do
        # segundo ponto e assim por diante. No IndexedTriangleStripSet uma lista informando
        # como conectar os vértices é informada em index, o valor -1 indica que a lista
        # acabou. A ordem de conexão será de 3 em 3 pulando um índice. Por exemplo: o
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. Cuidado com a orientação dos vértices, ou seja,
        # todos no sentido horário ou todos no sentido anti-horário, conforme especificado.
        invert = False
        
        for i in range(6, len(point)-2, 3):
            tri = []
            if not invert:
                tri.append([point[i],   point[i-3], point[i-6]])
                tri.append([point[i+1], point[i-2], point[i-5]])
                tri.append([point[i+2], point[i-1], point[i-4]])
            else: # If invert, invert p1 and p2
                tri.append([point[i-3],  point[i],    point[i-6]])
                tri.append([point[i-2],  point[i+1],  point[i-5]])
                tri.append([point[i-1],  point[i+2],  point[i-4]])

            tri.append([1,1,1])

            tri_M = np.array(tri)

            tri_M = np.matmul(GL.stack[-1],tri_M)
            
            # Transformações Mundo-camera
            tri_M = np.matmul(GL.Look_At,tri_M)
            z_list = tri_M[-2]

            # Transformações perspectiva
            tri_M = np.matmul(GL.camera_M_P,tri_M)
            
            # Transformações NDC
            tri_M = tri_M/tri_M[-1]
            
            # Transformaões mapping tela
            tri_M = np.matmul(GL.tela_M,tri_M)
            
            # Lista de pontos x,y
            tri_M = tri_M[:2,:].T.flatten().tolist()
            
            GL.triangleSet2D(tri_M,colors,z_list = z_list)

            if inverse:
                invert = not invert

    @staticmethod
    def box(size, colors):
        """Função usada para renderizar Boxes."""
        # A função box é usada para desenhar paralelepípedos na cena. O Box é centrada no
        # (0, 0, 0) no sistema de coordenadas local e alinhado com os eixos de coordenadas
        # locais. O argumento size especifica as extensões da caixa ao longo dos eixos X, Y
        # e Z, respectivamente, e cada valor do tamanho deve ser maior que zero. Para desenha
        # essa caixa você vai provavelmente querer tesselar ela em triângulos, para isso
        # encontre os vértices e defina os triângulos.
        pontos = []
        triangles = []
        for sx in range(-1,2,2):
            for sy in range(-1,2,2):
                for sz in range(-1,2,2):
                    pontos.append([size[0]*sx/2,size[1]*sy/2,size[2]*sz/2])
                    
        for p in GL.box_triangles:
            triangles+= pontos[p-1]
        
        GL.triangleSet(triangles, colors)

    @staticmethod
    def indexedFaceSet(coord, coordIndex, colorPerVertex, color, colorIndex,
                       texCoord, texCoordIndex, colors, current_texture):
        """Função usada para renderizar IndexedFaceSet."""
        # A função indexedFaceSet é usada para desenhar malhas de triângulos. Ela funciona de
        # forma muito simular a IndexedTriangleStripSet porém com mais recursos.
        # Você receberá as coordenadas dos pontos no parâmetro cord, esses
        # pontos são uma lista de pontos x, y, e z sempre na ordem. Assim coord[0] é o valor
        # da coordenada x do primeiro ponto, coord[1] o valor y do primeiro ponto, coord[2]
        # o valor z da coordenada z do primeiro ponto. Já coord[3] é a coordenada x do
        # segundo ponto e assim por diante. No IndexedFaceSet uma lista de vértices é informada
        # em coordIndex, o valor -1 indica que a lista acabou.
        # A ordem de conexão será de 3 em 3 pulando um índice. Por exemplo: o
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante.
        # Adicionalmente essa implementação do IndexedFace aceita cores por vértices, assim
        # se a flag colorPerVertex estiver habilitada, os vértices também possuirão cores
        # que servem para definir a cor interna dos poligonos, para isso faça um cálculo
        # baricêntrico de que cor deverá ter aquela posição. Da mesma forma se pode definir uma
        # textura para o poligono, para isso, use as coordenadas de textura e depois aplique a
        # cor da textura conforme a posição do mapeamento. Dentro da classe GPU já está
        # implementadado um método para a leitura de imagens.
        triangle = []
        triangle_colors = []
        tex_coords = []
        if current_texture:
            textura = current_texture[0]
            textura_img = gpu.GPU.load_texture(textura)

            
            for index in coordIndex:
                if index == -1:
                    pass
                else:
                    triangle+= coord[index*3:index*3+3]
                    
            for index in texCoordIndex:
                if index == -1:
                    pass
                else:
                    tex_coords += texCoord[index*2:index*2+2]  
                    
                    
            GL.triangleSet(triangle,textura_img, is_texture = True, texture_coordinates = tex_coords)
            
            

        elif colorPerVertex and color:
        
            for index in coordIndex:
                if index == -1:
                    pass
                else:
                    triangle+= coord[index*3:index*3+3]
                    triangle_colors += color[index*3:index*3+3]
            GL.triangleSet(triangle,triangle_colors)

            pass
        else:
            for index in coordIndex:
                if index == -1:
                    pass
                else:
                    triangle+= coord[index*3:index*3+3]
            GL.triangleSet(triangle,colors)
            

    @staticmethod
    def sphere(radius, colors):
        """Função usada para renderizar Esferas."""
        # A função sphere é usada para desenhar esferas na cena. O esfera é centrada no
        # (0, 0, 0) no sistema de coordenadas local. O argumento radius especifica o
        # raio da esfera que está sendo criada. Para desenha essa esfera você vai
        # precisar tesselar ela em triângulos, para isso encontre os vértices e defina
        # os triângulos.
        res = 5
        lista_andares = []
        #formando listas com os pontos em cada Z
        for z in np.arange(-radius,radius,radius/res):
            if z == -radius:
                continue
            raio_circulo = math.sqrt(radius**2 - z**2)
            lista_andar = []
            for x in np.arange(-raio_circulo,raio_circulo,raio_circulo/res):
                y = math.sqrt(raio_circulo**2 - x**2)
                ponto = [x,y,z]
                lista_andar.append(ponto)
                
            for i in range(len(lista_andar)):
                ponto = lista_andar[i]
                ponto_invertido = [-ponto[0],-ponto[1],ponto[2]]
                lista_andar.append(ponto_invertido)
            lista_andares.append(lista_andar)

        #montando os triangulos
        for i in range(len(lista_andares)-1):
            lista_desenha = []
            for j in range(len(lista_andares[0])):
                lista_desenha += lista_andares[i][j]
                lista_desenha += lista_andares[i+1][j]

            lista_desenha += lista_andares[i][0]
            lista_desenha += lista_andares[i+1][0]

            
            lista_tamanho = list(range(len(lista_andares[0])+1)) + [-1]
            GL.indexedTriangleStripSet(lista_desenha, lista_tamanho, colors,inverse=True)
        #primeiro andar
        for j in range(len(lista_andares[0])-1):
            lista_desenha = []
            lista_desenha += [0,0,-radius]
            lista_desenha += lista_andares[0][j+1]
            lista_desenha += lista_andares[0][j]
            
            
            GL.triangleSet(lista_desenha,colors)
        lista_desenha = []
        lista_desenha += [0,0,-radius]
        lista_desenha += lista_andares[0][0]
        lista_desenha += lista_andares[0][-1]
        
        GL.triangleSet(lista_desenha,colors)
        #ultimo andar
        for j in range(len(lista_andares[0])-1):
            lista_desenha = []
            lista_desenha += [0,0,radius]
            lista_desenha += lista_andares[-1][j+1]
            lista_desenha += lista_andares[-1][j]
            
            GL.triangleSet(lista_desenha,colors)
        lista_desenha = []
        lista_desenha += [0,0,radius]
        lista_desenha += lista_andares[-1][0]
        lista_desenha += lista_andares[-1][-1]
        
        GL.triangleSet(lista_desenha,colors)


        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("Sphere : radius = {0}".format(radius)) # imprime no terminal o raio da esfera
        print("Sphere : colors = {0}".format(colors)) # imprime no terminal as cores

    @staticmethod
    def navigationInfo(headlight):
        """Características físicas do avatar do visualizador e do modelo de visualização."""
        # O campo do headlight especifica se um navegador deve acender um luz direcional que
        # sempre aponta na direção que o usuário está olhando. Definir este campo como TRUE
        # faz com que o visualizador forneça sempre uma luz do ponto de vista do usuário.
        # A luz headlight deve ser direcional, ter intensidade = 1, cor = (1 1 1),
        # ambientIntensity = 0,0 e direção = (0 0 −1).

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("NavigationInfo : headlight = {0}".format(headlight)) # imprime no terminal

    @staticmethod
    def directionalLight(ambientIntensity, color, intensity, direction):
        """Luz direcional ou paralela."""
        # Define uma fonte de luz direcional que ilumina ao longo de raios paralelos
        # em um determinado vetor tridimensional. Possui os campos básicos ambientIntensity,
        # cor, intensidade. O campo de direção especifica o vetor de direção da iluminação
        # que emana da fonte de luz no sistema de coordenadas local. A luz é emitida ao
        # longo de raios paralelos de uma distância infinita.
        
        for i in range(len(direction)):
            direction[i] = direction[i]*-1

        GL.directional_light = {
            'Iia':ambientIntensity,
            'Ilrgb':color,
            'Ii':intensity,
            'L':np.array(direction)
        }

        GL.is_directional_light = True

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("DirectionalLight : ambientIntensity = {0}".format(ambientIntensity))
        print("DirectionalLight : color = {0}".format(color)) # imprime no terminal
        print("DirectionalLight : intensity = {0}".format(intensity)) # imprime no terminal
        print("DirectionalLight : direction = {0}".format(direction)) # imprime no terminal

    @staticmethod
    def pointLight(ambientIntensity, color, intensity, location):
        """Luz pontual."""
        # Fonte de luz pontual em um local 3D no sistema de coordenadas local. Uma fonte
        # de luz pontual emite luz igualmente em todas as direções; ou seja, é omnidirecional.
        # Possui os campos básicos ambientIntensity, cor, intensidade. Um nó PointLight ilumina
        # a geometria em um raio de sua localização. O campo do raio deve ser maior ou igual a
        # zero. A iluminação do nó PointLight diminui com a distância especificada.

        GL.point_light = {
            'Ia':ambientIntensity,
            'Ilrgb':color,
            'Ii':intensity,
            'location':np.array(location)
        }

        GL.is_point_light = True

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("PointLight : ambientIntensity = {0}".format(ambientIntensity))
        print("PointLight : color = {0}".format(color)) # imprime no terminal
        print("PointLight : intensity = {0}".format(intensity)) # imprime no terminal
        print("PointLight : location = {0}".format(location)) # imprime no terminal

    @staticmethod
    def fog(visibilityRange, color):
        """Névoa."""
        # O nó Fog fornece uma maneira de simular efeitos atmosféricos combinando objetos
        # com a cor especificada pelo campo de cores com base nas distâncias dos
        # vários objetos ao visualizador. A visibilidadeRange especifica a distância no
        # sistema de coordenadas local na qual os objetos são totalmente obscurecidos
        # pela névoa. Os objetos localizados fora de visibilityRange do visualizador são
        # desenhados com uma cor de cor constante. Objetos muito próximos do visualizador
        # são muito pouco misturados com a cor do nevoeiro.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("Fog : color = {0}".format(color)) # imprime no terminal
        print("Fog : visibilityRange = {0}".format(visibilityRange))

    @staticmethod
    def timeSensor(cycleInterval, loop):
        """Gera eventos conforme o tempo passa."""
        # Os nós TimeSensor podem ser usados para muitas finalidades, incluindo:
        # Condução de simulações e animações contínuas; Controlar atividades periódicas;
        # iniciar eventos de ocorrência única, como um despertador;
        # Se, no final de um ciclo, o valor do loop for FALSE, a execução é encerrada.
        # Por outro lado, se o loop for TRUE no final de um ciclo, um nó dependente do
        # tempo continua a execução no próximo ciclo. O ciclo de um nó TimeSensor dura
        # cycleInterval segundos. O valor de cycleInterval deve ser maior que zero.

        # Deve retornar a fração de tempo passada em fraction_changed

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("TimeSensor : cycleInterval = {0}".format(cycleInterval)) # imprime no terminal
        print("TimeSensor : loop = {0}".format(loop))

        # Esse método já está implementado para os alunos como exemplo
        epoch = time.time()  # time in seconds since the epoch as a floating point number.
        fraction_changed = (epoch % cycleInterval) / cycleInterval

        return fraction_changed

    @staticmethod
    def splinePositionInterpolator(set_fraction, key, keyValue, closed):
        """Interpola não linearmente entre uma lista de vetores 3D."""
        # Interpola não linearmente entre uma lista de vetores 3D. O campo keyValue possui
        # uma lista com os valores a serem interpolados, key possui uma lista respectiva de chaves
        # dos valores em keyValue, a fração a ser interpolada vem de set_fraction que varia de
        # zeroa a um. O campo keyValue deve conter exatamente tantos vetores 3D quanto os
        # quadros-chave no key. O campo closed especifica se o interpolador deve tratar a malha
        # como fechada, com uma transições da última chave para a primeira chave. Se os keyValues
        # na primeira e na última chave não forem idênticos, o campo closed será ignorado.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("SplinePositionInterpolator : set_fraction = {0}".format(set_fraction))
        print("SplinePositionInterpolator : key = {0}".format(key)) # imprime no terminal
        print("SplinePositionInterpolator : keyValue = {0}".format(keyValue))
        print("SplinePositionInterpolator : closed = {0}".format(closed))

        # Abaixo está só um exemplo de como os dados podem ser calculados e transferidos
        value_changed = [0.0, 0.0, 0.0]
        
        return value_changed

    @staticmethod
    def orientationInterpolator(set_fraction, key, keyValue):
        """Interpola entre uma lista de valores de rotação especificos."""
        # Interpola rotações são absolutas no espaço do objeto e, portanto, não são cumulativas.
        # Uma orientação representa a posição final de um objeto após a aplicação de uma rotação.
        # Um OrientationInterpolator interpola entre duas orientações calculando o caminho mais
        # curto na esfera unitária entre as duas orientações. A interpolação é linear em
        # comprimento de arco ao longo deste caminho. Os resultados são indefinidos se as duas
        # orientações forem diagonalmente opostas. O campo keyValue possui uma lista com os
        # valores a serem interpolados, key possui uma lista respectiva de chaves
        # dos valores em keyValue, a fração a ser interpolada vem de set_fraction que varia de
        # zeroa a um. O campo keyValue deve conter exatamente tantas rotações 3D quanto os
        # quadros-chave no key.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("OrientationInterpolator : set_fraction = {0}".format(set_fraction))
        print("OrientationInterpolator : key = {0}".format(key)) # imprime no terminal
        print("OrientationInterpolator : keyValue = {0}".format(keyValue))

        # Abaixo está só um exemplo de como os dados podem ser calculados e transferidos
        value_changed = [0, 0, 1, 0]

        return value_changed

    # Para o futuro (Não para versão atual do projeto.)
    def vertex_shader(self, shader):
        """Para no futuro implementar um vertex shader."""

    def fragment_shader(self, shader):
        """Para no futuro implementar um fragment shader."""