import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import CorobeuClass as crb
import csv

numSamples = 50
desiredPosition = [0,0]
executedPath = []
wheelsVelocities = []
orientation = []

try: 
    robot = crb.Corobeu(19999, 'robot01', 'motorL01', 'motorR01')
    for inst in range(numSamples):
        executedPath.append([])
        wheelsVelocities.append([])
        orientation.append([])
        executedPath[inst], wheelsVelocities[inst], orientation[inst] = robot.Aquire_Data()

finally:
    print('Fim da Aquisição')
    dfr01 = [executedPath, wheelsVelocities, orientation, desiredPosition]
    ir01 = ['Caminho Percorrido', 'Velocidades', 'Orientacao', 'Ponto Desejado']
    dfd = pd.DataFrame(dfr01, index=ir01)
    dfd.to_csv("dados100.csv", sep=';')
    dfd.head()
    print("Arquivo Gerado")
    