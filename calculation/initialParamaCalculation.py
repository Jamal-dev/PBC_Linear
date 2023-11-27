import math
from settings.simulationSettingsHexa import Names
###################################
#######  Calculating the geometrical parameters
# num_fibers = 2

# diameter of fiber
d_f = Names.DIA_FIBER # micro-meter
r_f = d_f/2.0
# volume of fiber = pi * (d_f/2)**2 * 2*a1
# volume of fibers = num_fibers * volume of fiber = 4 * a1 * pi * (d_f/2)**2
# volume of RVE = 2a1 * 2a2 * 2a3 = 8 * a1 * a2 * a3
# volume fraction of fiber = volume of fibers/ volume of RVE = pi/(2*a2*a3) * (d_f/2)**2
# volume fraction
volume_f = Names.VOL_FRAC_FIBER

# tan(60) = a3/a2 = sqrt(3)
# a3 = sqrt(3) * a2
a2 = math.pi/(2.0*math.sqrt(3.0) * volume_f) * (d_f/2.0)**2
a2 = math.sqrt(a2)
a3 = math.sqrt(3) * a2
# a1 can be chosen as arbitarely 
a1 = a2/4
Names.a1 = a1
Names.a2 = a2
Names.a3 = a3