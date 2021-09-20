#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# pylint: disable=invalid-name

"""
Biblioteca Gráfica / Graphics Library.

Desenvolvido por: <SEU NOME AQUI>
Disciplina: Computação Gráfica
Data:
"""

import time         # Para operações com tempo

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



    def triangleSet2D(vertices, colors):
        """Função usada para renderizar TriangleSet2D."""
        # Nessa função você receberá os vertices de um triângulo no parâmetro vertices,
        # esses pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o
        # valor da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto.
        # Já point[2] é a coordenada x do segundo ponto e assim por diante. Assuma que a
        # quantidade de pontos é sempre multiplo de 3, ou seja, 6 valores ou 12 valores, etc.
        # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet2D
        # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
        r = round((colors['diffuseColor'][0])*255)
        g = round((colors['diffuseColor'][1])*255)
        b = round((colors['diffuseColor'][2])*255)
        print(r,g,b)
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
                    gpu.GPU.draw_pixels([starter[0], starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])

                starterX = starter[0] -1
                while starterX >= lowerX:        
                    color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starterX, starter[1])            
                    if color_lv > 0:
                        gpu.GPU.draw_pixels([starterX, starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])
                    starterX -= 1


                while starterX <= higherX:
                    color_lv = TaDentro(x1, y1, x2, y2, x3, y3, starterX, starter[1])            
                    if color_lv > 0:
                        gpu.GPU.draw_pixels([starterX, starter[1]], gpu.GPU.RGB8, [int(r*color_lv), int(g*color_lv), int(b*color_lv)])
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
        GL.stack = []
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
        print("SETTTTTTTTTTTTTTTTTT")
        for i in range(0,len(point)-8,9):
            tri = []
            tri.append([point[i],point[i+3],point[i+6]])
            tri.append([point[i+1],point[i+4],point[i+7]])
            tri.append([point[i+2],point[i+5],point[i+8]])
            tri.append([1,1,1])
            tri_M = np.array(tri)
            
            # Transformações objeto-mundo
            for j in GL.stack:
                
                tri_M = np.matmul(j,tri_M)
            
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

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("TriangleSet : pontos = {0}".format(point)) # imprime no terminal pontos
        print("TriangleSet : colors = {0}".format(colors)) # imprime no terminal as cores

        # Exemplo de desenho de um pixel branco na coordenada 10, 10
        gpu.GPU.draw_pixels([10, 10], gpu.GPU.RGB8, [255, 255, 255])  # altera pixel

    @staticmethod
    def viewpoint(position, orientation, fieldOfView):
        """Função usada para renderizar (na verdade coletar os dados) de Viewpoint."""
        # Na função de viewpoint você receberá a posição, orientação e campo de visão da
        # câmera virtual. Use esses dados para poder calcular e criar a matriz de projeção
        # perspectiva para poder aplicar nos pontos dos objetos geométricos.

        # O print abaixo é só par"a vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("VIEWPOINT!!!!!!!!!")
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
        print("ENTREI NO TRANFORM IN")
        M_I = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        if translation:
            M_T = np.array([[1,0,0,translation[0]],
                                [0,1,0,translation[1]],
                                [0,0,1,translation[2]],
                                [0,0,0,      1      ]])
            M_I = np.matmul(M_T,M_I)
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
            
        GL.stack.append(M_I)
    @staticmethod
    def transform_out():
        print("OUTTTTTTTTT!!!")
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

            for j in GL.stack:
                
                tri_M = np.matmul(j,tri_M)
            
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

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("TriangleStripSet : pontos = {0} ".format(point), end='')
        for i, strip in enumerate(stripCount):
            print("strip[{0}] = {1} ".format(i, strip), end='')
        print("")
        print("TriangleStripSet : colors = {0}".format(colors)) # imprime no terminal as cores

        # Exemplo de desenho de um pixel branco na coordenada 10, 10
        gpu.GPU.draw_pixels([10, 10], gpu.GPU.RGB8, [255, 255, 255])  # altera pixel

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

            for j in GL.stack:
                
                tri_M = np.matmul(j,tri_M)
            
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
        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("IndexedTriangleStripSet : pontos = {0}, index = {1}".format(point, index))
        print("IndexedTriangleStripSet : colors = {0}".format(colors)) # imprime as cores

        # Exemplo de desenho de um pixel branco na coordenada 10, 10
        gpu.GPU.draw_pixels([10, 10], gpu.GPU.RGB8, [255, 255, 255])  # altera pixel

    @staticmethod
    def box(size, colors):
        """Função usada para renderizar Boxes."""
        # A função box é usada para desenhar paralelepípedos na cena. O Box é centrada no
        # (0, 0, 0) no sistema de coordenadas local e alinhado com os eixos de coordenadas
        # locais. O argumento size especifica as extensões da caixa ao longo dos eixos X, Y
        # e Z, respectivamente, e cada valor do tamanho deve ser maior que zero. Para desenha
        # essa caixa você vai provavelmente querer tesselar ela em triângulos, para isso
        # encontre os vértices e defina os triângulos.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("Box : size = {0}".format(size)) # imprime no terminal pontos
        print("Box : colors = {0}".format(colors)) # imprime no terminal as cores

        # Exemplo de desenho de um pixel branco na coordenada 10, 10
        gpu.GPU.draw_pixels([10, 10], gpu.GPU.RGB8, [255, 255, 255])  # altera pixel

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

        # Os prints abaixo são só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("IndexedFaceSet : ")
        if coord:
            print("\tpontos(x, y, z) = {0}, coordIndex = {1}".format(coord, coordIndex))
        print("colorPerVertex = {0}".format(colorPerVertex))
        if colorPerVertex and color and colorIndex:
            print("\tcores(r, g, b) = {0}, colorIndex = {1}".format(color, colorIndex))
        if texCoord and texCoordIndex:
            print("\tpontos(u, v) = {0}, texCoordIndex = {1}".format(texCoord, texCoordIndex))
        if current_texture:
            image = gpu.GPU.load_texture(current_texture[0])
            print("\t Matriz com image = {0}".format(image))
            print("\t Dimensões da image = {0}".format(image.shape))
        print("IndexedFaceSet : colors = {0}".format(colors))  # imprime no terminal as cores

        # Exemplo de desenho de um pixel branco na coordenada 10, 10
        gpu.GPU.draw_pixels([10, 10], gpu.GPU.RGB8, [255, 255, 255])  # altera pixel

    @staticmethod
    def sphere(radius, colors):
        """Função usada para renderizar Esferas."""
        # A função sphere é usada para desenhar esferas na cena. O esfera é centrada no
        # (0, 0, 0) no sistema de coordenadas local. O argumento radius especifica o
        # raio da esfera que está sendo criada. Para desenha essa esfera você vai
        # precisar tesselar ela em triângulos, para isso encontre os vértices e defina
        # os triângulos.

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
