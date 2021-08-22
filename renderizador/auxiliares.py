#calcula area do triangulo
def Area(x1, y1, x2, y2, x3, y3) :

    return abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))/2)
#calcula se p(x,y) está dentro do triangulo p1,p2,p3
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