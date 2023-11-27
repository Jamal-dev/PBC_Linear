import os
os.chdir(r'%s'%os.getcwd())
from settings.simulationSettingsHexa import Names
from calculation import initialParamaCalculation

from geometry.hexFiberArray import generateHexFiberArray
from materialProperties.isoModel import MatProp
from periodicCond.mechanicalLoading import pipline_3D_periodicity
from postProcess.post_process import runPostProcess

matProps = {"fiber": MatProp(E=Names.YOUNG_MODULI_FIBER,
                             nu= Names.POISSON_RATIO_FIBER,
                             name = "Fiber"),
            "matrix":MatProp(E=Names.YOUNG_MODULI_MAT,
                             nu= Names.POISSON_RATIO_MAT,
                             name = "Matrix")}

generateHexFiberArray(vf = Names.VOL_FRAC_FIBER, 
                      fiber_radius = Names.DIA_FIBER/2.0, 
                      interface_ratio = 0.0, 
                      depth = 2*Names.a1, 
                      n_rad = Names.MESH_DIVISION_R, 
                      n_depth = Names.MESH_DIVISION_Z, 
                      RVE_modelName = Names.MODEL_NAME,
                      matPropties=matProps)



import abaqus
import abaqusConstants as const
import part
import assembly
import interaction
import mesh


# from abaqusConstants import *

# Gettting a model object
createdModel = abaqus.mdb.models[Names.MODEL_NAME]
abaqus.session.journalOptions.setValues(replayGeometry=const.COORDINATE,recoverGeometry=const.COORDINATE)

# step
createdModel.StaticLinearPerturbationStep(name= Names.STEP_NAME, previous=
                                                            'Initial')

# output requests
createdModel.fieldOutputRequests['F-Output-1'].setValues(variables=(
    'NE','LE','E', 'S', 'U', 'IVOL'))

# Adding constraint equations
pipline_3D_periodicity(Model=createdModel,Names=Names)
# create job
job_name = Names.JOB_NAME
abaqus.mdb.Job(atTime=None, contactPrint=const.OFF, description='', echoPrint=const.OFF, 
        explicitPrecision=const.SINGLE, getMemoryFromAnalysis=True, historyPrint=const.OFF, 
        memory=90, memoryUnits=const.PERCENTAGE, model=Names.MODEL_NAME, modelPrint=const.OFF, 
        multiprocessingMode=const.DEFAULT, name=job_name, nodalOutputPrecision=const.SINGLE, 
        numCpus=1, numGPUs=0, queue=None, resultsFormat=const.ODB, scratch='', type=
        const.ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)

abaqus.mdb.jobs[job_name].submit(consistencyChecking=const.OFF)

abaqus.mdb.jobs[job_name].waitForCompletion()
runPostProcess(createdModel)