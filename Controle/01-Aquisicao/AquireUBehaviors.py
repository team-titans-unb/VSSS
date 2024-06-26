import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import CorobeuClass as crb

numSamples = 50
desiredPosition = [0.0, 0.0]
xd = []
yd = []
executedPath = []
xe = []
ye = []
ve = []
vd = []
wheelsVelocities = []
orientation = []

try: 
    robot = crb.Corobeu(19999, 'robot01', 'motorL01', 'motorR01')
    for inst in range(numSamples):
        print(inst)
        # executedPath.append([])
        # wheelsVelocities.append([])
        orientation.append([])
        xd.append(desiredPosition[0])
        yd.append(desiredPosition[1])
        executedPath, wheelsVelocities, orientation[inst] = robot.Aquire_Data()
        xe.append(executedPath[0])
        ye.append(executedPath[1])
        print(wheelsVelocities)
        ve.append(wheelsVelocities[0])
        vd.append(wheelsVelocities[1])

finally:
    print('Fim da Aquisição')
    dfr01 = [xe, ye, orientation, ve, vd, xd, yd]
    ir01 = ["X", "Y", "Gama", "Vr", "Vl", "Xd", "Yd"]
    dfd = pd.DataFrame(dfr01, index=ir01)
    dfd.to_csv("dados101.csv", sep=';')
    dfd.head()
    print("Arquivo Gerado")
    