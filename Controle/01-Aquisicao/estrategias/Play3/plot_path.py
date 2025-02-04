import csv
import matplotlib.pyplot as plt
from DrawField import draw_field

def plot_trajectories(csv_filename):
    blue_x, blue_y = [], []
    yellow_x, yellow_y = [], []
    ball_x, ball_y = [], []

    with open(csv_filename, mode='r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            if row["blue_robot_x"] and row["blue_robot_y"]:
                blue_x.append(float(row["blue_robot_x"]) / 10)  # Convertendo mm para cm
                blue_y.append(float(row["blue_robot_y"]) / 10)
            
            if row["yellow_robot_x"] and row["yellow_robot_y"]:
                yellow_x.append(float(row["yellow_robot_x"]) / 10)  # Convertendo mm para cm
                yellow_y.append(float(row["yellow_robot_y"]) / 10)
            
            if row["ball_x"] and row["ball_y"]:
                ball_x.append(float(row["ball_x"]) / 10)  # Convertendo mm para cm
                ball_y.append(float(row["ball_y"]) / 10)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_field(ax)
    plt.plot(blue_x, blue_y, 'b-', label="Robô Azul")
    plt.plot(yellow_x, yellow_y, 'y-', label="Robô Amarelo")
    plt.plot(ball_x, ball_y, 'r-', label="Bola")
    
    plt.xlabel("Posição X (cm)")
    plt.ylabel("Posição Y (cm)")
    plt.title("Trajetória dos Robôs e da Bola no Campo")
    plt.legend()
    plt.grid()
    plt.show()

# Nome do arquivo CSV gerado anteriormente
csv_filename = "atacante_01.csv"
plot_trajectories(csv_filename)
