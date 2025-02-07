import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from DrawField import draw_field

def load_trajectory_data(csv_filename):
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
    
    return blue_x, blue_y, yellow_x, yellow_y, ball_x, ball_y

def animate_trajectory(csv_filename, output_gif="trajectory.gif"):
    blue_x, blue_y, yellow_x, yellow_y, ball_x, ball_y = load_trajectory_data(csv_filename)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_field(ax)
    
    blue_line, = ax.plot([], [], 'b-', label="Robô Azul")
    yellow_line, = ax.plot([], [], 'y-', label="Robô Amarelo")
    ball_line, = ax.plot([], [], 'r-', label="Bola")
    legend = ax.legend()
    
    def update(frame):
        start_frame = max(0, frame-50)
        blue_line.set_data(blue_x[start_frame:frame], blue_y[start_frame:frame])
        yellow_line.set_data(yellow_x[start_frame:frame], yellow_y[start_frame:frame])
        ball_line.set_data(ball_x[start_frame:frame], ball_y[start_frame:frame])
        legend.get_texts()[0].set_text(f"Robô Azul")
        legend.get_texts()[1].set_text(f"Robô Amarelo")
        legend.get_texts()[2].set_text(f"Bola - Frame: {frame}")
        return blue_line, yellow_line, ball_line, legend
    
    ani = animation.FuncAnimation(fig, update, frames=len(blue_x), interval=100, blit=True)
    ani.save(output_gif, writer='pillow', fps=10)
    plt.close()
    print(f"GIF salvo como {output_gif}")

# Nome do arquivo CSV gerado anteriormente
if __name__ == "__main__":
    for i in range(43):
        csv_filename = f"teste_{i+1}.csv"
        gif_filename = f"teste_{i+1}.gif"
        animate_trajectory(csv_filename, gif_filename)
