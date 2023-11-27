import json
# from pathlib import Path

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
    with open(r'.\materialProperties.json') as f:
        prop_list = json.load(f)
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

class Names:
    # Here which one you will choose, it will generate the material model and also
    # it will generate the stiffness matrix
    DIA_FIBER = inProp.d_f
    VOL_FRAC_FIBER = inProp.V_f
    YOUNG_MODULI_FIBER = inProp.Ef
    YOUNG_MODULI_MAT = inProp.Em
    POISSON_RATIO_FIBER = inProp.nu_f
    POISSON_RATIO_MAT = inProp.nu_m
    MESH_DIVISION_R = inProp.mesh_r
    MESH_DIVISION_Z = inProp.mesh_z
    ALL_ORIGINAL_ELEMENTS_SET = 'all-org-elements'
    RELATIVE_PERIODIC_POSITION_TOLERANCE = 1e-3
    SORTED_SET_SUFFIX ='_sorted'
    POSITIVE_SET_SYMBOL = 'p'
    NEGATIVE_SET_SYMBOL = 'n'
    SUBTRACT_SET_OPERATION = 'S'
    POSITIVE_X_SET = 'X%s'%POSITIVE_SET_SYMBOL
    NEGATIVE_X_SET = 'X%s'%NEGATIVE_SET_SYMBOL
    POSITIVE_Y_SET = 'Y%s'%POSITIVE_SET_SYMBOL
    NEGATIVE_Y_SET = 'Y%s'%NEGATIVE_SET_SYMBOL
    POSITIVE_Z_SET = 'Z%s'%POSITIVE_SET_SYMBOL
    NEGATIVE_Z_SET = 'Z%s'%NEGATIVE_SET_SYMBOL
    POSITIVE_Y_SUB_NEG_X =           ('%s_%s_%s')%(POSITIVE_Y_SET,SUBTRACT_SET_OPERATION,NEGATIVE_X_SET)
    NEGATIVE_Y_SUB_NEG_X =           ('%s_%s_%s')%(NEGATIVE_Y_SET,SUBTRACT_SET_OPERATION,NEGATIVE_X_SET)
    POSITIVE_Z_SUB_X_SUB_Y =         ('%s_%s_%s_%s_%s')%(POSITIVE_Z_SET,SUBTRACT_SET_OPERATION,'X',SUBTRACT_SET_OPERATION,'Y')
    NEGATIVE_Z_SUB_X_SUB_Y =         ('%s_%s_%s_%s_%s')%(NEGATIVE_Z_SET,SUBTRACT_SET_OPERATION,'X',SUBTRACT_SET_OPERATION,'Y')
    POSITIVE_Z_SUB_NEG_X_SUB_NEG_Y = ('%s_%s_%s_%s_%s')%(POSITIVE_Z_SET,SUBTRACT_SET_OPERATION,NEGATIVE_X_SET,SUBTRACT_SET_OPERATION,NEGATIVE_Y_SET)
    NEGATIVE_Z_SUB_NEG_X_SUB_NEG_Y = ('%s_%s_%s_%s_%s')%(NEGATIVE_Z_SET,SUBTRACT_SET_OPERATION,NEGATIVE_X_SET,SUBTRACT_SET_OPERATION,NEGATIVE_Y_SET)
    POSITIVE_Y_SUB_POS_X = ('%s_%s_%s')%(POSITIVE_Y_SET,SUBTRACT_SET_OPERATION,POSITIVE_X_SET)
    NEGATIVE_Y_SUB_POS_X = ('%s_%s_%s')%(NEGATIVE_Y_SET,SUBTRACT_SET_OPERATION,POSITIVE_X_SET)
    STEP_NAME = 'Step-1'
    PREFIX_CONSTRAINTS = 'constraint'
    JOB_NAME = 'hexa_model_micro_scale'
    AUTOGENERATED_STRING = ''
    MODEL_NAME = 'Model-1'
    NUM_STEPS_EACH_MODEL = [1,1,1,1,1,1]
    PART_NAME  = 'HexPackPart'
    INSTANCE_NAME  = '%s-1'%PART_NAME
    STEP_NAME      = 'Step-1'
    coordSysName = 'Assembly_global'
    ODB_TEMP_GLOBAL_CSYS = 'globalCYS'
    LOAD_CASES_NAMES = ['Column-1','Column-2','Column-3','Column-4','Column-5','Column-6']
    APPLIED_STRAIN_KEYS = [11,12,13,21,22,23,31,32,33]
    a1=None
    a2=None
    a3=None
    AUTOGENERATED_PREFIX = 'RVE_'
    SURFACE_ELEMENT_SECTION_NAME = '%sProjected_Faces' % AUTOGENERATED_PREFIX
    SURFACE_ELEMENTS_SET = '%sSurface_Elements' % AUTOGENERATED_PREFIX
    MECHANICAL_NORM_REF_NODE = 'RP-Normal'
    MECHANICAL_SHEAR_REF_NODE = 'RP-Shear'
    MECHANICAL_PIN_VERTEX = '%sPinNode' % AUTOGENERATED_PREFIX
    COPIED_FACETS_SUFFIX = '_%sFACETCOPY' % AUTOGENERATED_PREFIX
    TIE_PREFIX = '%s_Tie_' % AUTOGENERATED_PREFIX
    REDUCED_SET_SUFFIX = '_%sRED' % AUTOGENERATED_PREFIX
    PARTITION_FACE_SKETCH_NAME = 'hexaInside'
    