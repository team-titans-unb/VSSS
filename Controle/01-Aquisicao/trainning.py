import pandas as pd
import Bioinspired as bia
import matplotlib.pyplot as plt
import os

def main():
    folder_name = '\Controle\Training_data'
    file_name = 'dados101.csv'
    file_path = os.path.join(folder_name, file_name)
    # print(file_path)

    data = pd.read_csv(file_name, delimiter=";", index_col=0)  # Definimos a primeira coluna como índice

    # Renomeia as colunas para facilitar o acesso
    data.index = ["X", "Y", "Gama", "Vr", "Vl", "Xd", "Yd"]

    # Separa as entradas e saídas desejadas
    inputs = data.loc[["X", "Y", "Gama", "Xd", "Yd"]].values
    outputs = data.loc[["Vr", "Vl"]].values

    # Cria o objeto BioinspiredAlgorithms e executa o PSO
    bio_alg = bia.Bioinspired_algorithms()
    fitnessVector, weigths_biases = bio_alg.PSO(inputs, (100*outputs))

    fig_folder = '\Controle\Figures'
    fig_name = '1stBehaviorFitness.pdf'
    fig_path = os.path.join(fig_folder, fig_name)
    os.makedirs(fig_folder, exist_ok=True)
    plt.figure()
    plt.plot(fitnessVector, label='MSE Convergence for the Neural Network')
    plt.title('MSE Convergence for the 1st Behavior Trainning')
    plt.xlabel('Number of Iterations')
    plt.ylabel('Mean Square Error')
    plt.legend()
    plt.savefig(fig_path, format='pdf')

    wb_folder = '/Controle/weights'
    wb_file = '1stBehaviorWB101.csv'
    wb_path = os.path.join(wb_folder, wb_file)
    os.makedirs(wb_folder, exist_ok=True)
    dfr01 = [weigths_biases]
    # ir01 = ['Weights 0:38 and Biases 39:47']
    dfd = pd.DataFrame(dfr01)
    dfd.to_csv(wb_path, sep=',', index=False)
    print("Arquivo Gerado")

if __name__ == "__main__":
    main()