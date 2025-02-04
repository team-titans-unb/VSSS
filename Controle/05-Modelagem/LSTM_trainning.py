import os
import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.utils import plot_model
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt

# Configuração
SEQUENCE_LENGTH = 5  # Número de frames usados como entrada
DATASET_DIR = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\estrategias_temporal"  # Substituir pelo caminho real
OUTPUT_DIR = r"D:\\Documents\\Titans\\VSSS\\VSSS\\Controle\\05-Modelagem\\resultados"

# Carregar arquivos CSV
def load_dataset(dataset_dir):
    all_files = [f for f in os.listdir(dataset_dir) if f.endswith(".csv")]
    data = []
    labels = []
    
    for file in all_files:
        df = pd.read_csv(os.path.join(dataset_dir, file))
        df.drop(columns=["timestamp", "frame_number"], inplace=True, errors='ignore')  # Removendo colunas irrelevantes
        df.fillna(df.mean(), inplace=True)  # Lidando com valores NaN
        
        strategy = file.split("_")[0]  # Extrai a estratégia do nome do arquivo
        data.append(df.values)
        labels.append(strategy)
    
    return data, labels

# Carregar dados
raw_data, raw_labels = load_dataset(DATASET_DIR)

# Converter labels para valores numéricos
label_encoder = LabelEncoder()
y = label_encoder.fit_transform(raw_labels)

# Normalizar os dados
scaler = StandardScaler()
X_normalized = [scaler.fit_transform(sequence) for sequence in raw_data]

# Criar sequências para LSTM
def create_sequences(data, labels, seq_length):
    X, y = [], []
    for sequence, label in zip(data, labels):
        for i in range(len(sequence) - seq_length):
            X.append(sequence[i:i+seq_length])
            y.append(label)
    return np.array(X), np.array(y)

X, y = create_sequences(X_normalized, y, SEQUENCE_LENGTH)

# Dividir em treino e teste
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Criar o modelo LSTM
model = keras.Sequential([
    keras.layers.LSTM(64, return_sequences=True, input_shape=(SEQUENCE_LENGTH, X.shape[2])),
    keras.layers.LSTM(32),
    keras.layers.Dense(16, activation='relu'),
    keras.layers.Dense(len(np.unique(y)), activation='softmax')
])

# Compilar o modelo
model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

# Treinar
epochs = 50
model.fit(X_train, y_train, epochs=epochs, batch_size=8, validation_data=(X_test, y_test))

# Avaliação
loss, accuracy = model.evaluate(X_test, y_test)
print(f'Accuracy: {accuracy * 100:.2f}%')

# Salvar o modelo
model.save(os.path.join(OUTPUT_DIR, "lstm_teste.h5"))

# Salvar imagem do modelo
plot_model(model, to_file=os.path.join(OUTPUT_DIR, "lstm_teste.png"), show_shapes=True, show_layer_names=True)

# Gerar e exibir a matriz de confusão
y_pred = np.argmax(model.predict(X_test), axis=1)
conf_matrix = confusion_matrix(y_test, y_pred)
display = ConfusionMatrixDisplay(confusion_matrix=conf_matrix, display_labels=label_encoder.classes_)
display.plot(cmap=plt.cm.Blues)
plt.title("Matriz de Confusão")
plt.savefig(os.path.join(OUTPUT_DIR, "confusion_matrix.png"))
plt.show()

# Exibir pesos e vieses das camadas
for i, layer in enumerate(model.layers):
    layer_weights = layer.get_weights()
    if len(layer_weights) == 2:  # Apenas camadas com pesos e viéses
        weights, biases = layer_weights
        print(f"Camada {i} - Pesos: {weights.shape}, Viéses: {biases.shape}")
    elif len(layer_weights) == 1:  # Algumas camadas podem ter apenas pesos
        weights = layer_weights[0]
        print(f"Camada {i} - Apenas Pesos: {weights.shape}")
    else:
        print(f"Camada {i} - Sem Pesos ou Viéses")