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
            

    def triangleSet2D(vertices, colors, z_list = [0, 0, 0]):
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
            
            a,b,c = plane_eq(x1, y1, z1, x2, y2, z2, x3, y3, z3)
            
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
                    #gpu.GPU.draw_pixels([starter[0], starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])
                    
                    # Baricentro
                    alpha,beta,gama,Z = Baricentro(x1, y1, z1, x2, y2, z2, x3, y3, z3, starter[0], starter[1])
                    
                    # Interpolar cor com perspectiva
                    if condition == 'multi_color':
                        color = QueCorDeus(alpha,c1,z1,beta,c2,z2,gama,c3,z3,Z)
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
                        
                        # Interpolar cor com perspectiva
                        if condition == 'multi_color':
                            color = QueCorDeus(alpha,c1,z1,beta,c2,z2,gama,c3,z3,Z)
                            
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
                        
                        # Interpolar cor com perspectiva
                        if condition == 'multi_color':
                            color = QueCorDeus(alpha,c1,z1,beta,c2,z2,gama,c3,z3,Z)
                        
                        z = calc_z(starterX,starter[1],a,b,c)
                        point = [starterX, starter[1], z]
                        
                        GL.draw_pixel(point,color)
                        
                    
                    starterX += 1

                starter[1] = starter[1]-1
            
    @staticmethod
    def setup(width, height, near=0.01, far=1000):
        """Definr parametros para câmera de razão de aspecto, plano próximo e distante."""
        GL.width = width
        GL.height = height
        GL.ascpect = width/height
        GL.near = near
        GL.far = far
        
        
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
    def triangleSet(point, colors):
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
        if 'diffuseColor' in colors:
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
            
            if condition == 'multi_color':
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
        
        print(GL.Look_At)
        
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
    def indexedTriangleStripSet(point, index, colors):
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
        
        if colorPerVertex and color:
        
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
            
    # Para o futuro (Não para versão atual do projeto.)
    def vertex_shader(self, shader):
        """Para no futuro implementar um vertex shader."""

    def fragment_shader(self, shader):
        """Para no futuro implementar um fragment shader."""
