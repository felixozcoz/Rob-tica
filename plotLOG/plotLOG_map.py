import csv
import matplotlib.pyplot as plt
import numpy as np
import os
from geometry import Vector2, Matrix2, Transform
from plot_robot import dibrobot

# Leemos los ficheros de la carpeta 'logs'
#list_of_files = glob.glob('./logs/*')
# Buscamos el más reciente, el de la última ejecución
#latest_file = max(list_of_files, key=os.path.getctime)
from ReMapLib import Map

print("---------------------------------------------------")
rMap = Map("../p4/maps/mapa2.txt", [0,0], [2,6], neighborhood=4)
ltow = Matrix2.transform(Vector2(20, 20, 0), 90)
def plot_log(log_name,rMap):
    # create a new figure and set it as current axis
    current_fig = plt.figure()
    rMap.current_ax = current_fig.add_subplot(111)
    rMap._drawGrid()
    rMap._drawCostMatrix()
    with open(f"{log_name}.csv", "r") as log_file:
        # Creamos el lector de fichero de CSV
        reader = csv.reader(log_file) 
        # Saltamos la cabecera
        next(reader, None)
        # Iteramos sobre cada fila del log, que corresponden a los valores de la odometría
        for row in reader:
            gpos        = ltow * Vector2(np.float32(row[1]), np.float32(row[2]), 1)
            # Ploteamos cada nueva posición de la odometría
            # dibrobot([np.float32(row[1])+20,np.float32(row[2])+20,np.float32(row[3])], 'b', 'g')
            dibrobot([gpos.x,gpos.y,np.float32(row[3])], 'b', 'g')
    plt.savefig(f"{log_name}.png")
    plt.close()

#        Ocho                               Bicicleta buena 1                  Bicicleta buena 2
#logs = ["ODOMETRYLOG_18-54-29_2024-02-28", "ODOMETRYLOG_18-41-58_2024-02-28", "ODOMETRYLOG_18-37-36_2024-02-28"]
#for log in logs:
#    plot_log(log)

#last_log = os.listdir("./")[-1].split('.')[-2]
last_log = "ODOMETRYLOG_2024-04-26_10-18-05"
plot_log(last_log,rMap)


