import glob
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from plot_robot import dibrobot

list_of_files = glob.glob('./logs/*') # * means all if need specific format then *.csv
latest_file = max(list_of_files, key=os.path.getctime)

with open(latest_file) as file: 
      
    # Create reader object by passing the file  
    # object to reader method 
    reader = csv.reader(file) 
    next(reader, None)  # skip the headers

    # Iterate over each row in the csv  
    # file using reader object 
    for row in reader: 
        dibrobot([np.float32(row[1]),np.float32(row[2]),np.float32(row[3])], 'b', 'g')
    
plt.show()