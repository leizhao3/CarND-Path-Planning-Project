from math import sqrt, exp
from matplotlib import pyplot as plt
import numpy as np

def logistic(x):
    """
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """
    return 2.0 / (1 + np.exp(-x)) - 1.0

x = np.linspace(-25,25,1000)
y = logistic(x)

plt.plot(x,y)
plt.show()