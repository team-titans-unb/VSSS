import os
import pandas as pd
import re
import math

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

            # Calcula as médias dos valores
            rpm_right_mean = df["RPM_Right"].mean()
            rpm_left_mean = df["RPM_Left"].mean()
            velocity_x_mean = df["Velocity_X"].mean()
            velocity_y_mean = df["Velocity_Y"].mean()
            velocity_total_mean = df["Velocity_Total"].mean()

            # Coleta os dados para os três arquivos de saída
            data_right.append([pwm_right, rpm_right_mean])
            data_left.append([pwm_left, rpm_left_mean])
            data_speed.append([pwm_right, pwm_left, velocity_x_mean, velocity_y_mean, velocity_total_mean])
            
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
