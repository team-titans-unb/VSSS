import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tensorflow as tf
from tensorflow import keras
from sklearn.preprocessing import StandardScaler
import joblib  # Para salvar e carregar o scaler
from DrawField import draw_field

# Caminho da pasta contendo os arquivos CSV
DATASET_DIR = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\estrategias\\treino"
MODEL_PATH = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\resultados\\lstm_futebol_robotico.h5"
SCALER_PATH = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\resultados\\scaler.pkl"  # Caminho do scaler salvo


# Carregar modelo treinado
model = tf.keras.models.load_model(MODEL_PATH)
scaler = joblib.load(SCALER_PATH)

# Função para carregar dados da trajetória
def load_trajectory_data(csv_filename):
    blue_x, blue_y, yellow_x, yellow_y, ball_x, ball_y = [], [], [], [], [], []
    features = []
    
    with open(csv_filename, mode='r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            blue_x.append(float(row["blue_robot_x"]) / 10 if row["blue_robot_x"] else 0.0)
            blue_y.append(float(row["blue_robot_y"]) / 10 if row["blue_robot_y"] else 0.0)
            yellow_x.append(float(row["yellow_robot_x"]) / 10 if row["yellow_robot_x"] else 0.0)
            yellow_y.append(float(row["yellow_robot_y"]) / 10 if row["yellow_robot_y"] else 0.0)
            ball_x.append(float(row["ball_x"]) / 10 if row["ball_x"] else 0.0)
            ball_y.append(float(row["ball_y"]) / 10 if row["ball_y"] else 0.0)
            
            # Coletando features para predição
            feature_row = [
                float(row["blue_robot_x"]) / 10 if row["blue_robot_x"] else 0.0,    
                float(row["blue_robot_y"]) / 10 if row["blue_robot_y"] else 0.0,
                float(row["blue_robot_orientation"]) if row["blue_robot_orientation"] else 0.0,
                float(row["yellow_robot_x"]) / 10 if row["yellow_robot_x"] else 0.0,
                float(row["yellow_robot_y"]) / 10 if row["yellow_robot_y"] else 0.0,
                float(row["yellow_robot_orientation"]) if row["yellow_robot_orientation"] else 0.0,
                float(row["ball_x"]) / 10 if row["ball_x"] else 0.0,
                float(row["ball_y"]) / 10 if row["ball_y"] else 0.0,
                float(row["ball_speed_x"]) if row["ball_speed_x"] else 0.0,
                float(row["ball_speed_y"]) if row["ball_speed_y"] else 0.0,
                float(row["ball_speed"]) if row["ball_speed"] else 0.0,
                float(row["ball_angle"]) if row["ball_angle"] else 0.0
            ]
            features.append(feature_row)
    
    return blue_x, blue_y, yellow_x, yellow_y, ball_x, ball_y, np.array(features)

# Função para gerar GIF com estratégias previstas
def animate_trajectory(csv_filename, output_gif="trajectory.gif"):
    blue_x, blue_y, yellow_x, yellow_y, ball_x, ball_y, features = load_trajectory_data(csv_filename)
    
    # Normalizando os dados antes de passar para o modelo
    scaler = StandardScaler()
    features_normalized = scaler.fit_transform(features)
    
    # Criando sequências para o modelo LSTM
    SEQUENCE_LENGTH = 50
    X = [features_normalized[i-SEQUENCE_LENGTH:i] for i in range(SEQUENCE_LENGTH, len(features_normalized))]
    X = np.array(X)
    
    # Fazendo previsões das estratégias
    y_pred = model.predict(X)
    strategy_labels = np.argmax(y_pred, axis=1)  # Convertendo para labels numéricas
    
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_field(ax)
    
    blue_line, = ax.plot([], [], 'b-', label="Robô Azul")
    yellow_line, = ax.plot([], [], 'y-', label="Robô Amarelo")
    ball_line, = ax.plot([], [], 'r-', label="Bola")
    strategy_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')
    
    def update(frame):
        start_frame = max(0, frame-50)
        blue_line.set_data(blue_x[start_frame:frame], blue_y[start_frame:frame])
        yellow_line.set_data(yellow_x[start_frame:frame], yellow_y[start_frame:frame])
        ball_line.set_data(ball_x[start_frame:frame], ball_y[start_frame:frame])
        legend = ax.legend()

        if frame >= SEQUENCE_LENGTH:
            estrategia = strategy_labels[frame-SEQUENCE_LENGTH]
        else:
            estrategia = "N/A"
            # strategy_text.set_text(f"Estratégia prevista: {strategy_labels[frame-SEQUENCE_LENGTH]}")
        legend.get_texts()[0].set_text(f"Robô Azul - Estratégia: {estrategia}")
        legend.get_texts()[1].set_text(f"Robô Amarelo")
        legend.get_texts()[2].set_text(f"Bola - Frame: {frame}")
        return blue_line, yellow_line, ball_line, strategy_text
    
    ani = animation.FuncAnimation(fig, update, frames=len(blue_x), interval=100, blit=True)
    ani.save(output_gif, writer='pillow', fps=10)
    plt.close()
    print(f"GIF salvo como {output_gif}")

# Processar todos os arquivos CSV da pasta
if __name__ == "__main__":
    for file in os.listdir(DATASET_DIR):
        if file.endswith(".csv"):
            csv_path = os.path.join(DATASET_DIR, file)
            gif_filename = os.path.join(DATASET_DIR, file.replace(".csv", ".gif"))
            animate_trajectory(csv_path, gif_filename)
