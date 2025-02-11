import os
import pandas as pd

def extract_last_rows(output_file="sistema.csv"):
    # Obtém todos os arquivos CSV que começam com "R" no diretório atual
    csv_files = [f for f in os.listdir() if f.startswith("R") and f.endswith(".csv")]
    
    all_last_rows = []
    
    for file in csv_files:
        df = pd.read_csv(file)
        
        # Garante que o arquivo não está vazio
        if not df.empty:
            last_row = df.iloc[-1]  # Obtém a última linha
            # last_row["Arquivo"] = file  # Adiciona o nome do arquivo para referência
            all_last_rows.append(last_row)
    
    if all_last_rows:
        result_df = pd.DataFrame(all_last_rows)
        result_df.to_csv(output_file, index=False)
        print(f"Arquivo {output_file} criado com sucesso!")
    else:
        print("Nenhum dado foi encontrado nos arquivos CSV.")

# Executa o script
extract_last_rows()