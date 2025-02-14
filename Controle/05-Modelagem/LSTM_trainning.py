import os
import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.utils import plot_model
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay, classification_report
import matplotlib.pyplot as plt
import time

# Configuração
SEQUENCE_LENGTH = 5  # Número de frames usados como entrada
DATASET_DIR = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\estrategias\\treino"
OUTPUT_DIR = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\resultados"

# Carregar arquivos CSV
def load_dataset(dataset_dir):
    all_files = [f for f in os.listdir(dataset_dir) if f.endswith(".csv")]
    data = []
    labels = []
    
    for file in all_files:
        df = pd.read_csv(os.path.join(dataset_dir, file))
        df.drop(columns=["timestamp", "frame_number"], inplace=True, errors='ignore')
        df.fillna(df.mean(numeric_only=True), inplace=True)  # Lidando com valores NaN
        
        # Criando dataset para o robô azul
        strategy_blue = df["blue_robot_strategy"].values
        features_blue = df.drop(columns=["blue_robot_strategy", "yellow_robot_strategy"], errors='ignore').copy()
        data.append(features_blue.values)
        labels.append(strategy_blue)
        
        # Criando dataset para o robô amarelo (trocando as colunas)
        strategy_yellow = df["yellow_robot_strategy"].values
        features_yellow = df.copy()
        features_yellow.rename(columns={
            "yellow_robot_x": "blue_robot_x", "yellow_robot_y": "blue_robot_y", "yellow_robot_orientation": "blue_robot_orientation",
            "blue_robot_x": "yellow_robot_x", "blue_robot_y": "yellow_robot_y", "blue_robot_orientation": "yellow_robot_orientation"
        }, inplace=True)
        features_yellow.drop(columns=["blue_robot_strategy", "yellow_robot_strategy"], inplace=True, errors='ignore')
        data.append(features_yellow.values)
        labels.append(strategy_yellow)
    
    return data, labels

# Carregar dados
data, labels = load_dataset(DATASET_DIR)

# Normalizar os dados
scaler = StandardScaler()
X_normalized = [scaler.fit_transform(sequence) for sequence in data]

# Certificar-se de que labels são arrays NumPy
labels = [np.array(lbl) for lbl in labels]

# Criar sequências para LSTM
def create_sequences(data, labels, seq_length):
    X, y = [], []
    for sequence, label in zip(data, labels):
        sequence = np.array(sequence, dtype=np.float32)  # Converte para float32
        
        if len(sequence) <= seq_length:
            continue  # Ignorar sequências curtas
        
        for i in range(len(sequence) - seq_length):
            X.append(sequence[i:i+seq_length])
            y.append(label[i + seq_length])
    
    return np.array(X, dtype=np.float32), np.array(y, dtype=np.int32)


X, y = create_sequences(X_normalized, labels, SEQUENCE_LENGTH)
y = np.array(y, dtype=np.int32)
label_encoder = LabelEncoder()

# Ajustar e transformar os rótulos
y = label_encoder.fit_transform(y)
# Dividir em treino e teste
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Criar o modelo LSTM
model = keras.Sequential([
    keras.layers.LSTM(64, return_sequences=True, input_shape=(SEQUENCE_LENGTH, X.shape[2])),
    keras.layers.LSTM(32),
    keras.layers.Dense(16, activation='relu'),
    keras.layers.Dense(len(set(y)), activation='softmax')  # Corrigindo o mapeamento das estratégias
])

# Compilar o modelo
model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

# Treinar
epochs = 50
model.fit(X_train, y_train, epochs=epochs, batch_size=8, validation_data=(X_test, y_test))

# Avaliação
loss, accuracy = model.evaluate(X_test, y_test)
print(f'Accuracy: {accuracy * 100:.2f}%')

# Relatório de classificação
y_pred = np.argmax(model.predict(X_test), axis=1)
print("Relatório de Classificação:")
print(classification_report(y_test, y_pred))

# Análise de casos perdidos
misclassified_idx = np.where(y_test != y_pred)[0]
print("Casos Perdidos:")
for idx in misclassified_idx[:5]:  # Mostrando os primeiros 5 erros
    print(f"Verdadeiro: {y_test[idx]}, Previsto: {y_pred[idx]}")

# Medindo tempo de inferência
start_time = time.time()
model.predict(X_test[:10])
end_time = time.time()
print(f"Tempo médio de inferência: {(end_time - start_time) / 10:.6f} segundos")

# Salvar o modelo
model.save(os.path.join(OUTPUT_DIR, "lstm_futebol_robotico.h5"))

# Salvar imagem do modelo
plot_model(model, to_file=os.path.join(OUTPUT_DIR, "lstm_model.png"), show_shapes=True, show_layer_names=True)

# Gerar e exibir a matriz de confusão
conf_matrix = confusion_matrix(y_test, y_pred)
display = ConfusionMatrixDisplay(confusion_matrix=conf_matrix)
display.plot(cmap=plt.cm.Blues)
plt.title("Matriz de Confusão")
plt.savefig(os.path.join(OUTPUT_DIR, "confusion_matrix.png"))
plt.show()
