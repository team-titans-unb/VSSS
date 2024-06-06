import pandas as pd
import Bioinspired as bia

def main():
    file_path = "dados23.csv"
    data = pd.read_csv(file_path, delimiter=";", index_col=0)  # Definimos a primeira coluna como índice

    # Renomeia as colunas para facilitar o acesso
    data.index = ["X", "Y", "Gama", "Vr", "Vl", "Xd", "Yd"]

    # Separa as entradas e saídas desejadas
    inputs = data.loc[["X", "Y", "Gama", "Xd", "Yd"]].values
    outputs = data.loc[["Vr", "Vl"]].values

    # Cria o objeto BioinspiredAlgorithms e executa o PSO
    bio_alg = bia.Bioinspired_algorithms()
    bio_alg.PSO(inputs, outputs)
    print(bio_alg.fit_vector)

if __name__ == "__main__":
    main()