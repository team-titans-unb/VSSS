import os
import pandas as pd

def swap_robot_positions_in_csv(directory="."):
    # Listar todos os arquivos que começam com "goleiro" e têm extensão .csv
    csv_files = [f for f in os.listdir(directory) if f.startswith("zagueiro") and f.endswith(".csv")]
    print("Files read")
    for file in csv_files:
        file_path = os.path.join(directory, file)
        
        # Ler o arquivo CSV
        df = pd.read_csv(file_path)
        
        # Trocar os valores das colunas mantendo os nomes das colunas
        df["blue_robot_x"], df["yellow_robot_x"] = df["yellow_robot_x"], df["blue_robot_x"].copy()
        df["blue_robot_y"], df["yellow_robot_y"] = df["yellow_robot_y"], df["blue_robot_y"].copy()
        df["blue_robot_orientation"], df["yellow_robot_orientation"] = df["yellow_robot_orientation"], df["blue_robot_orientation"].copy()
        
        # Salvar o arquivo CSV modificado
        df.to_csv(file_path, index=False)
        print(f"Arquivo processado: {file}")

if __name__ == "__main__":
    swap_robot_positions_in_csv()
