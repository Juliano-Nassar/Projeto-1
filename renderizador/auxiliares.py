import numpy as np

#calcula area do triangulo
def Area(x1, y1, x2, y2, x3, y3) :

    return abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))/2)
#calcula se p(x,y) estÃ¡ dentro do triangulo p1,p2,p3
def TaDentro(x1, y1, x2, y2, x3, y3, x, y):
    x = x +0.5
    y = y+0.5
    #area do triangulo p1,p2,p3
    A = Area(x1, y1, x2, y2, x3, y3)

    #areas menores (com p)
    A1 = Area(x, y, x2, y2, x3, y3)
    A2 = Area(x1, y1, x, y, x3, y3)
    A3 = Area(x1, y1, x2, y2, x, y)
    #if (A1+A2+A3 == A) :
    if abs((A1+A2+A3)-A) < 0.00001 :
        return True
    else:
        return False

def Baricentro(x1, y1, z1, x2, y2, z2, x3, y3, z3, x, y):
    #alpha = 1
    #beta  = 2
    #gama  = 3

    #area do triangulo p1,p2,p3
    A = Area(x1, y1, x2, y2, x3, y3)

    #areas menores (com p)
    A1 = Area(x, y, x2, y2, x3, y3)
    A2 = Area(x1, y1, x, y, x3, y3)
    A3 = Area(x1, y1, x2, y2, x, y)
    
    #if (A1+A2+A3 == A) :
    alpha = A1/A
    beta = A2/A
    gama = A3/A
    Z = 1/((alpha/z1) + (beta/z2) + (gama/z3))
    
    return alpha,beta,gama,Z

def QueCorDeus(alpha,c1,z1,beta,c2,z2,gama,c3,z3,Z):
    _5eh10 = []

    for i in range(3):
        contaDificil = Z*(((alpha*c1[i])/z1) + ((beta*c2[i])/z2) + ((gama*c3[i])/z3))
        _5eh10.append(round(abs(contaDificil*255)))


    return _5eh10

# Resultado de [a b c] = [[x1 y1 1],[x2 y2 1],[x3 y3 1]]^-1 * [z1, z2, z3]
# Estas são as matrizes que representam a equação do plano substituindo os pontos
# https://www.wolframalpha.com/input/?i=inverse+%5B%5Bx1%2Cy1%2C1%5D%2C%5Bx2%2Cy2%2C1%5D%2C%5Bx3%2Cy3%2C1%5D%5D*%5Bz1%2Cz2%2Cz3%5D
def plane_eq(x1, y1, z1, x2, y2, z2, x3, y3, z3):
    a = ((y2 - y3)*z1)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3) + ((-y1 + y3)*z2)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3) + ((y1 - y2)*z3)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3)
    b = ((-x2 + x3)*z1)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3) + ((x1 - x3)*z2)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3) + ((-x1 + x2)*z3)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3)
    c = ((-x3*y2 + x2*y3)*z1)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3) + ((x3*y1 - x1*y3)*z2)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3) + ((-x2*y1 + x1*y2)*z3)/(-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3)
    return a,b,c

def calc_z(x,y,a,b,c):
    return a*x + y*b +c



# Link aula de luzes:
# http://jogos-digitais.s3-website-us-east-1.amazonaws.com/courses/computacao-grafica/aula12-calculo_de_iluminacao.html

def difuse_light(Kd, I, r, n,l):
    Ld = Kd*(I/(r**2)*max([0,n.dot(l)]))
    return Ld

def specular_light(Ks, I, r, n, l, v, p):
    # Normaliza h
    h = v+l
    h = (h)/np.linalg.norm(h)

    Ls = Ks*(I/r**2)*(max([0,n.dot(h)]))**p

    return Ls

def ambient_light(Ka, Ia):
    La = Ka*Ia
    return La

def calc_light(Kd, Ks, Ka, I, r, n, l, v, p, Ia):
    # Cálcula luzes separadamentes
    Ld = difuse_light(Kd, I, r, n,l)
    Ls = specular_light(Ks, I, r, n, l, v, p)
    La = ambient_light(Ka, Ia)

    L = La + Ls + Ld

    return L

# Interpolação
def Hermit_Catmull_Rom(p0,p1,p2,p3):

    HCR_matrix = np.matrix([[ -1/2 , 3/2, -3/2 ,  1/2 ],
                            [   1 , -5/2,   2 ,  -1/2 ],
                            [ -0.5,   0 ,  0.5,     0 ],
                            [   0 ,   1 ,   0 ,     0 ])
    
    p = np.matrix([p0,
                   p1,
                   p2,
                   p3])
    
    result = np.matmul(HCR_matrix, p)

    return result