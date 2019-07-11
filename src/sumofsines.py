
import matplotlib.pyplot as plt
import numpy as np

def generate_sumofsines_fn(frequencies, magnitudes=None, phases=None):
    frequencies = np.asarray(frequencies)
    if magnitudes is None:
        magnitudes = np.ones(frequencies.shape)
    if phases is None:
        phases = np.zeros(frequencies.shape)
    
    sinelist = []    
    for index in range(len(frequencies)):
        freq = frequencies[index]
        mag = magnitudes[index]
        phase = phases[index]
        
        sinelist.append((freq, mag, phase))

    print(sinelist)
        
    sumofsines = lambda t: np.sum(np.asarray([ tup[1]*np.sin((tup[0]*np.asarray(t))+tup[2]) for tup in sinelist ]), axis = 0)
    return sumofsines


if __name__ == "__main__":
    t = np.linspace(0, 6, 100)
    sos123 = generate_sumofsines_fn([1,2,3])
    
    plt.plot(t, sos123(t), t, np.sin(t),t,np.sin(2*t),t,np.sin(3*t))
    plt.show()