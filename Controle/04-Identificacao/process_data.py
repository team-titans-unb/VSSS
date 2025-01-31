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

            # Verifica se há dados suficientes para calcular deslocamento e tempo
            if len(df) > 1:
                # Obtém os primeiros e últimos valores de posição e tempo
                x_start, y_start = df.iloc[0]["X"], df.iloc[0]["Y"]
                x_end, y_end = df.iloc[-1]["X"], df.iloc[-1]["Y"]
                time_start, time_end = df.iloc[0]["Timestamp"], df.iloc[-1]["Timestamp"]

                # Calcula deslocamento total e tempo decorrido
                displacement_total = math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
                time_elapsed = time_end - time_start

                # Calcula velocidade total do robô
                velocity_total = displacement_total / time_elapsed if time_elapsed > 0 else 0
            else:
                velocity_total = 0

            # Calcula as médias dos valores de RPM
            rpm_right_mean = df["RPM_Right"].mean()
            rpm_left_mean = df["RPM_Left"].mean()

            # Coleta os dados para os três arquivos de saída
            data_right.append([pwm_right, rpm_right_mean])
            data_left.append([pwm_left, rpm_left_mean])
            data_speed.append([pwm_right, pwm_left, velocity_total])


# Converte os dados para DataFrames do Pandas
df_right = pd.DataFrame(data_right, columns=["PWM_Right", "RPM_Right_Mean"])
df_left = pd.DataFrame(data_left, columns=["PWM_Left", "RPM_Left_Mean"])
df_speed = pd.DataFrame(data_speed, columns=["PWM_Right", "PWM_Left", "Velocity_Total"])

# Salva os arquivos de saída
df_right.to_csv("pwm_rpm_direita.csv", index=False)
df_left.to_csv("pwm_rpm_esquerda.csv", index=False)
df_speed.to_csv("pwm_velocidade.csv", index=False)

print("Arquivos gerados com sucesso:")
print("- pwm_rpm_direita.csv")
print("- pwm_rpm_esquerda.csv")
print("- pwm_velocidade.csv")
