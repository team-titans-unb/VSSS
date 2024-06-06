class Calculadora_Estatistica_Erro:
    
    def __init__ (self):
        self.lista_medias = []
        self.erro_medio = 0
        self.variancia = 0
        self.desvioPadrao = 0
        
        ## Essa parte é nescessaria pois faz o programa não calcular duas vezes a mesma coisa
        self.contMedia = 0 # Verifica se a média foi calculada ,se for = 0 nao foi, se = 1 foi
        self.contVariancia = 0 # Verifica se a variancia foi calculada ,se for = 0 nao foi, se = 1 foi
        self.contDesvioPadrao = 0 # Verifica se o desvio padrão foi calculada ,se for = 0 nao foi, se = 1 foi
        
    def retornar_erro (self,posicaoDesejada , posicaoAtual):
        erro = abs(posicaoDesejada - posicaoAtual)
        self.lista_medias.append(erro)
        return erro
    
    def retornar_erro_medio (self):
        if(self.contMedia == 0):
            self.erro_medio = sum(self.lista_medias)/len(self.lista_medias)
            return self.erro_medio
        else:
            return self.erro_medio
        
    def retornar_variancia (self):
        
        if ( self.contVariancia == 0 ):
            if (self.contMedia == 0 ):
                self.retornar_erro_medio()
                delta = 0
                for i in  self.lista_medias:
                    delta = delta + (i - self.erro_medio)**2
                self.contVariancia = 1
                self.variancia = delta/len(self.lista_medias)
                return self.variancia
            else:
                delta = 0
                for i in self.lista_media:
                    delta = delta + (i - self.erro_medio)**2
                self.contVariancia = 1
                self.variancia = delta
                return self.variancia
        else:
            return self.variancia

            
        
    def retornar_desvio_padrao (self):
        
        if (self.contDesvioPadrao == 0 ):
            if (self.contVariancia == 0 ):
                self.retornar_variancia()
                self.desvioPadrao = self.variancia ** (1/2)
                self.contDesvioPadrao = 1
                return self.desvioPadrao
            else:
                self.desvioPadrao = self.variancia ** (1/2)
                self.contDesvioPadrao = 1
                return self.desvioPadrao
        else:
            return self.desvioPadrao
    
    def limpar_calculadora (self):
        self.lista_medias = []
        self.erro_medio = 0
        self.variancia = 0
        self.desvioPadrao = 0
        self.contMedia = 0 
        self.contVariancia = 0 
        self.contDesvioPadrao = 0 
        





    

        
        
            
            
            
            