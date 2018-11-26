import math

class Vetor():
    x = 0
    y = 0

    def __init__(self,arg0,arg1):
        self.x = arg0
        self.y = arg1
    def vetor_distancia(self,t0,t1):
        return Vetor(-(t0[2]) + (t1[2]), -(t0[1]) + (t1[1]))

    def toString(self): 
        return "("+ str(self.x) + ","+ str(self.y) +")"
    
    def set(self,arg0,arg1):
        self.x = arg0
        self.y = arg1
    
    def inverte(self):
        return Vetor(self.y, self.x)
    
    def magnitude(self): 
        return (math.sqrt(self.x*self.x + self.y*self.y))
    
    def normalizar(self): 
        m = self.magnitude()
        if (m != 0):
            return Vetor(self.x/m,self.y/m)
        return Vetor(self.x,self.y)
    
    def limitar(self,maxi):
        if (self.magnitude() > maxi):
            self.normalizar()
            self.multiplica(maxi)
            
    def soma(self,outro):
        return Vetor(self.x + outro.x, self.y + outro.y)
    
    def subtrai(self,outro):
        return Vetor(self.x - outro.x, self.y - outro.y)
    
    def multiplica(self,scalar):
        return Vetor(self.x * scalar, self.y * scalar)
    
    def divide(self,scalar):
        if (scalar != 0):
            return Vetor(self.x / scalar, self.y / scalar)
        return Vetor(0,0)
    
    