import os
import pandas as pd
import re

# Pasta onde os arquivos CSV estão localizados (use "." se estiver no mesmo diretório)
FOLDER_PATH = "."

# Inicializa listas para armazenar os dados combinados
data_right = []
data_left = []
data_speed = []

# Padrão para extrair os valores de PWM do nome do arquivo (ex: "R100L50.csv")
file_pattern = re.compile(r"R(-?\d+)L(-?\d+)\.csv")

# Percorre todos os arquivos na pasta
for filename in os.listdir(FOLDER_PATH):
    if filename.endswith(".csv"):
        match = file_pattern.search(filename)
        print(f"{match}")
        if match:
            pwm_right = int(match.group(1))
            pwm_left = int(match.group(2))

            # Lê o CSV
            file_path = os.path.join(FOLDER_PATH, filename)
            df = pd.read_csv(file_path)

            # Adiciona a coluna de PWM ao DataFrame
            df["PWM_Right"] = pwm_right
            df["PWM_Left"] = pwm_left

            # Coleta os dados para os três arquivos de saída
            data_right.extend(df[["PWM_Right", "RPM_Right"]].values.tolist())
            data_left.extend(df[["PWM_Left", "RPM_Left"]].values.tolist())
            data_speed.extend(df[["PWM_Right", "PWM_Left", "Velocity_X", "Velocity_Y", "Velocity_Total"]].values.tolist())

# Converte os dados para DataFrames do Pandas
df_right = pd.DataFrame(data_right, columns=["PWM_Right", "RPM_Right"])
df_left = pd.DataFrame(data_left, columns=["PWM_Left", "RPM_Left"])
df_speed = pd.DataFrame(data_speed, columns=["PWM_Right", "PWM_Left", "Velocity_X", "Velocity_Y", "Velocity_Total"])

# Salva os arquivos de saída
df_right.to_csv("pwm_rpm_direita.csv", index=False)
df_left.to_csv("pwm_rpm_esquerda.csv", index=False)
df_speed.to_csv("pwm_velocidade.csv", index=False)

print("Arquivos gerados com sucesso:")
print("- pwm_rpm_direita.csv")
print("- pwm_rpm_esquerda.csv")
print("- pwm_velocidade.csv")
