import csv
import numpy as np
import matplotlib.pyplot as plt
from plot_robot import dibrobot

# Leemos los ficheros de la carpeta 'logs'
#list_of_files = glob.glob('./logs/*')
# Buscamos el más reciente, el de la última ejecución
#latest_file = max(list_of_files, key=os.path.getctime)

def plot_log(log_name):
    """
        Plot a received log
    """
    with open(f"./logs/{log_name}.csv", "r") as log_file:
        # Creamos el lector de fichero de CSV
        reader = csv.reader(log_file) 
        # Saltamos la cabecera
        next(reader, None)
        # Iteramos sobre cada fila del log, que corresponden a los valores de la odometría
        for row in reader:
            # Ploteamos cada nueva posición de la odometría
            dibrobot([np.float32(row[1]),np.float32(row[2]),np.float32(row[3])], 'b', 'g')
    plt.savefig(f"{log_name}.png")
    plt.close()

#        Ocho                               Bicicleta buena 1                  Bicicleta buena 2
logs = ["ODOMETRYLOG_18-54-29_2024-02-28", "ODOMETRYLOG_18-41-58_2024-02-28", "ODOMETRYLOG_18-37-36_2024-02-28"]
for log in logs:
    plot_log(log)
