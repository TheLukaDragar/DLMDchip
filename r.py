import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

with open('one.txt') as f:
    data = f.readlines()



np_data = np.array([x.strip() for x in data[0]])


#plot resposive

plt.plot(np_data)
plt.show()
