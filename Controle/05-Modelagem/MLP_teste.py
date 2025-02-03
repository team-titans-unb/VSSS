import os
import glob
import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.utils import plot_model
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder, StandardScaler

# Diretório onde estão os arquivos CSV
DATASET_DIR = DATASET_DIR = r"D:\Documents\Titans\VSSS\VSSS\Controle\05-Modelagem\estrategias"
output_dir = r"D:\Documents\Titans\VSSS\VSSS\Controle\05-Modelagem\resultados"


# Carregar todos os arquivos CSV na pasta
def load_dataset(dataset_dir):
    all_files = glob.glob(os.path.join(dataset_dir, "*.csv"))
    data = []
    labels = []
    
    for file in all_files:
        df = pd.read_csv(file)
        print(f"Lido arquivo: {file}")
        # Extrair o nome do arquivo para identificar a estratégia
        strategy = os.path.basename(file).split("_")[0]  # Ex: "atacante_02.csv" -> "atacante"
        
        data.append(df.values[0])  # Pegamos apenas a primeira linha, pois cada CSV tem um único estado
        labels.append(strategy)
    
    return np.array(data), np.array(labels)

# Carregar os dados
X, y = load_dataset(DATASET_DIR)

# Normalizar os dados (exceto a coluna de estratégia)
scaler = StandardScaler()
X = scaler.fit_transform(X)

# Transformar os rótulos (estratégia) em valores numéricos
label_encoder = LabelEncoder()
y = label_encoder.fit_transform(y)

# Dividir em treino e teste
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Criar o modelo MLP
model = keras.Sequential([
    keras.layers.Dense(64, activation='relu', input_shape=(X_train.shape[1],)),
    keras.layers.Dense(32, activation='relu'),
    keras.layers.Dense(len(np.unique(y)), activation='softmax')
])

# Compilar o modelo
model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

# Treinar o modelo
print("Iniciando processo de treinamento")
epochs = 50
model.fit(X_train, y_train, epochs=epochs, batch_size=8, validation_data=(X_test, y_test))

# Avaliar o modelo
loss, accuracy = model.evaluate(X_test, y_test)
print(f'Accuracy: {accuracy * 100:.2f}%')

model.summary()
model.save(os.path.join(output_dir, "mlp_de_teste.h5"))

for i, layer in enumerate(model.layers):
    weights, biases = layer.get_weights()
    print(f"Camada {i}:")
    print(f"  Pesos:\n{weights}")
    print(f"  Viéses:\n{biases}\n")
plot_model(model, to_file=os.path.join(output_dir, "mlp_teste.png"), show_shapes=True, show_layer_names=True)
