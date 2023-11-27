import json
from pathlib import Path

class InputProp:
    def __init__(self,Ef,Em,nu_f,nu_m,d_f,V_f,mesh_r,mesh_z):
        self.Ef     = Ef   
        self.Em     = Em   
        self.nu_f   = nu_f 
        self.nu_m   = nu_m 
        self.d_f    = d_f  
        self.V_f    = V_f  
        self.mesh_r = mesh_r
        self.mesh_z = mesh_z
    def __repr__(self):
        text = "Ef=%f, "%self.Ef
        text += "Em=%f, \n"%self.Em
        text += "nu_f=%f, "%self.nu_f
        text += "nu_m=%f\n, "%self.nu_m
        text += "d_f=%f, \n"%self.d_f
        text += "V_f=%f, \n"%self.V_f
        text += "mesh_r=%i, "%self.mesh_r
        text += "mesh_z=%i. \n"%self.mesh_z
        return text

def readProp():
    with open(Path(r'.\materialProperties.json')) as f:
        prop_list = json.load(f)
    print(prop_list)
    Ef   = float(prop_list['modulus']["Ef"])
    Em   = float(prop_list['modulus']["Em"])
    nu_f = float(prop_list['poisson_ratio']["nu_f"])
    nu_m = float(prop_list['poisson_ratio']["nu_m"])
    d_f  = float(prop_list['diameter_fiber'])
    V_f  = float(prop_list['volume_fraction_fiber'])
    mesh_r = int(prop_list['num_mesh_elements_r'])
    mesh_z = int(prop_list['num_mesh_eleemnts_z'])
    return InputProp(Ef,Em,nu_f,nu_m,d_f,V_f,mesh_r,mesh_z)
inProp = readProp()
print(inProp)
