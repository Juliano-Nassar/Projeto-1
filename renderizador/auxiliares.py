#calcula area do triangulo
def Area(x1, y1, x2, y2, x3, y3) :

    return abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))/2)
#calcula se p(x,y) está dentro do triangulo p1,p2,p3
def TaDentro(x1, y1, x2, y2, x3, y3, x, y):
    sub_p1 = [x+0.25,y+0.25]
    sub_p2 = [x+0.75,y+0.25]
    sub_p3 = [x+0.25,y+0.75]
    sub_p4 = [x+0.75,y+0.75]
    ps = [sub_p1, sub_p2, sub_p3, sub_p4]
    
    S2_Luciano_e_Orfalli = 0
    for p in ps:
        x = p[0]
        y = p[1]
        #area do triangulo p1,p2,p3
        A = Area(x1, y1, x2, y2, x3, y3)

        #areas menores (com p)
        A1 = Area(x, y, x2, y2, x3, y3)
        A2 = Area(x1, y1, x, y, x3, y3)
        A3 = Area(x1, y1, x2, y2, x, y)
        #if (A1+A2+A3 == A) :
        
        if abs((A1+A2+A3)-A) < 0.00001 :
            S2_Luciano_e_Orfalli += 1
    
    return S2_Luciano_e_Orfalli/len(ps)