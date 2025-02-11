import os
import pandas as pd

def process_csv_files():
    # Obtém todos os arquivos CSV que começam com "R" no diretório atual
    csv_files = [f for f in os.listdir() if f.startswith("R") and f.endswith(".csv")]
    
    for file in csv_files:
        df = pd.read_csv(file)
        
        # Garante que o arquivo tem pelo menos 20 linhas
        if len(df) >= 20:
            averages = df.tail(20).mean()
            
            # Adiciona uma nova linha com os valores médios
            df.loc[len(df)] = averages
            
            # Salva o arquivo atualizado
            df.to_csv(file, index=False)
            print(f"Média adicionada a {file}")
        else:
            print(f"{file} tem menos de 20 linhas, ignorado.")

# Executa o processamento
process_csv_files()