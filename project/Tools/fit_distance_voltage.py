#!/usr/bin/env python3

import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

voltage = np.array([2.9970647052455854,
                    2.723989767489559,
                    2.3384451825728263,
                    2.0171484568622198,
                    1.765443741133438,
                    1.561932098519248,
                    1.3985669927472237,
                    1.26732741184715,
                    1.061059144594032,
                    0.9270661295928071,
                    0.8144880041476985,
                    0.7367122647690558,
                    0.6589333348526369,
                    0.5247201727448307,
                    0.42798306736022873,
                    0.3713988798933441,
                    0.31213145015639254])

cm = np.array([3.5048217002147917,
               3.9726274648328666,
               4.996516616434688,
               5.978482101652813,
               7.04733048843715,
               7.988192731271259,
               9.015486642471757,
               10.000364632888747,
               12.013890234105713,
               14.028523407722243,
               16.04348475093865,
               18.01607119457153,
               20.03156581338773,
               25.00686307465289,
               29.98273463271783,
               34.959221508782534,
               40.021483714013875])


def f(x, a, b, c):
    return a / ((x + b) ** c)


fit = curve_fit(f, voltage, cm)[0]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(voltage, cm, linestyle='', marker='o')
voltagei = np.linspace(*ax.get_xlim())
cmi = f(voltagei, fit[0], fit[1], fit[2])
ax.plot(voltagei, cmi)
ax.grid()
fig.savefig('distance_voltage.pdf')

print(f'{fit[0]}/(voltage + {fit[1]})**{fit[2]}')
