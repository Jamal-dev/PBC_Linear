from settings.simulationSettingsHexa import Names
from post_process_helper_functions import matmul, call_different_homo_method, calc_comp_C_matmul
from materialModelCreation import generateAllHomogenizationScenerios

def runPostProcess(Model):
    print('Abaqus/Standard Stress Tensor Order:')
    print('Average stresses Global CSYS: 11-22-33-12-13-23')
    C, C_mat = calc_comp_C_matmul(Names=Names)
    C_diff = call_different_homo_method(Names)

    generateAllHomogenizationScenerios(Model, nameSuffix='MyAvgMethod', Names=Names, cMatrix=C_mat)
    # generateAllHomogenizationScenerios(model, nameSuffix='DiffAvgMethod', Names=Names, cMatrix=C_diff)