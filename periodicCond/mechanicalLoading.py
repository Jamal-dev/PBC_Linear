from pair_nodes_helper_functions import applyMechanicalPBC, create_applied_loadings,set_difference,nodes_sorting_faces,getAllNodesList
import mesh
import abaqusConstants as const
import abaqus
import numpy as np
# from mechanicalLoading import createBaseSets
def createBaseSets(Model,Names):
    """
        Create basic sets for RVE
            
        Inputs:
            Model - A model object from a Model Database
            Names - Class for global properties
    """
    eps = 1e-6
    a = abaqus.mdb.models[Names.MODEL_NAME].rootAssembly
    allNodesList = getAllNodesList(Model)
    face_nodes = {}
    face_nodes[Names.POSITIVE_X_SET] = []
    face_nodes[Names.NEGATIVE_X_SET] = []
    face_nodes[Names.POSITIVE_Y_SET] = []
    face_nodes[Names.NEGATIVE_Y_SET] = []
    face_nodes[Names.POSITIVE_Z_SET] = []
    face_nodes[Names.NEGATIVE_Z_SET] = []
    # allNodes = a.instances[Names.INSTANCE_NAME].nodes

    for allNodes in allNodesList:
        if len(allNodes)==0:
            continue
        # postive X set = 2a1, (-a2, a2), (-a3,a3)
        face_nodes[Names.POSITIVE_X_SET].extend(    allNodes.getByBoundingBox(2*Names.a1-eps    ,-Names.a2-eps,-Names.a3-eps-eps  , 2*Names.a1 +eps, Names.a2+eps, Names.a3+eps   ))
        # negative X set = 0, (-a2, a2), (-a3,a3)
        face_nodes[Names.NEGATIVE_X_SET].extend(    allNodes.getByBoundingBox(0-eps   ,-Names.a2-eps,-Names.a3-eps  ,0 +eps, Names.a2+eps, 2*Names.a3+eps   ))
        face_nodes[Names.POSITIVE_Y_SET].extend(    allNodes.getByBoundingBox(0-eps   , Names.a2-eps,-Names.a3-eps  , 2*Names.a1 +eps, Names.a2+eps, Names.a3+eps   ))
        face_nodes[Names.NEGATIVE_Y_SET].extend(    allNodes.getByBoundingBox(0-eps   ,-Names.a2-eps,-Names.a3-eps  , 2*Names.a1 +eps,-Names.a2+eps, Names.a3+eps   ))
        face_nodes[Names.POSITIVE_Z_SET].extend(    allNodes.getByBoundingBox(0-eps   ,-Names.a2-eps,Names.a3-eps ,   2*Names.a1 +eps, Names.a2+eps, Names.a3+eps   ))
        face_nodes[Names.NEGATIVE_Z_SET].extend(    allNodes.getByBoundingBox(0-eps   ,-Names.a2-eps,-Names.a3-eps  , 2*Names.a1 +eps, Names.a2+eps, -Names.a3+eps   ))
    face_nodes[Names.POSITIVE_Y_SUB_NEG_X] = mesh.MeshNodeArray(set_difference(face_nodes[Names.POSITIVE_Y_SET],face_nodes[Names.NEGATIVE_X_SET]))
    face_nodes[Names.NEGATIVE_Y_SUB_NEG_X] = mesh.MeshNodeArray(set_difference(face_nodes[Names.NEGATIVE_Y_SET],face_nodes[Names.NEGATIVE_X_SET]))
    face_nodes[Names.POSITIVE_Y_SUB_POS_X] = mesh.MeshNodeArray(set_difference(face_nodes[Names.POSITIVE_Y_SET],face_nodes[Names.POSITIVE_X_SET]))
    face_nodes[Names.NEGATIVE_Y_SUB_POS_X] = mesh.MeshNodeArray(set_difference(face_nodes[Names.NEGATIVE_Y_SET],face_nodes[Names.POSITIVE_X_SET]))
    face_nodes[Names.POSITIVE_Z_SUB_X_SUB_Y] = mesh.MeshNodeArray(set_difference(
                        set_difference(
                                    set_difference(set_difference(face_nodes[Names.POSITIVE_Z_SET],face_nodes[Names.NEGATIVE_X_SET]),face_nodes[Names.POSITIVE_X_SET]),face_nodes[Names.POSITIVE_Y_SET])
                                    , face_nodes[Names.NEGATIVE_Y_SET])
                                                            )
    face_nodes[Names.NEGATIVE_Z_SUB_X_SUB_Y] = mesh.MeshNodeArray(set_difference(
                        set_difference(
                                    set_difference(set_difference(face_nodes[Names.NEGATIVE_Z_SET],face_nodes[Names.NEGATIVE_X_SET]),face_nodes[Names.POSITIVE_X_SET]),face_nodes[Names.POSITIVE_Y_SET])
                                    , face_nodes[Names.NEGATIVE_Y_SET])
                                                            )
    face_nodes[Names.POSITIVE_Z_SUB_NEG_X_SUB_NEG_Y] = mesh.MeshNodeArray(set_difference(
                                    set_difference(face_nodes[Names.POSITIVE_Z_SET],face_nodes[Names.NEGATIVE_X_SET]),face_nodes[Names.NEGATIVE_Y_SET]))
    face_nodes[Names.NEGATIVE_Z_SUB_NEG_X_SUB_NEG_Y] = mesh.MeshNodeArray(set_difference(
                                    set_difference(face_nodes[Names.NEGATIVE_Z_SET],face_nodes[Names.NEGATIVE_X_SET]),face_nodes[Names.NEGATIVE_Y_SET]))
    # sort face nodes coordinates
    for k,v in face_nodes.items():
        searchStr = k.split('_')[0]
        if   'x' in searchStr.lower():
            face_nodes[k] = nodes_sorting_faces(face_nodes[k]  ,i=2,j=2,k=1) # normal x-axis 
        elif 'y' in searchStr.lower():
            face_nodes[k] = nodes_sorting_faces(face_nodes[k]  ,i=0,j=2,k=1) # normal y-axis
        elif 'z' in searchStr.lower():
            face_nodes[k] = nodes_sorting_faces(face_nodes[k]  ,i=0,j=1,k=2) # normal z-axis 
        else:
            raise ValueError("This %s is not part of the X,Y, and Z sets "%k)
    for name, nodes in face_nodes.items():
        set_nodes = []
        for n in nodes:
            set_nodes.append(n.label)
        a.SetFromNodeLabels(name=str(name), nodeLabels=((Names.INSTANCE_NAME, tuple(set_nodes)), ), unsorted=True)



    



def create_referencePoints(Model,Names):
    """
        Create 9 reference points which corresponds to eps11, eps12 ...
            
        Inputs:
            Model - A model object from a Model Database
            Names - Class for global properties
    """
    a = Model.rootAssembly
    # creating reference points
    if not 'RP-1' in a.features.keys():
        a.ReferencePoint(point=(3*Names.a1, -Names.a2,   -Names.a3   ))
        a.ReferencePoint(point=(3*Names.a1,  0.0 , -Names.a3  ))
        a.ReferencePoint(point=(3*Names.a1,  Names.a2,   -Names.a3   ))
        a.ReferencePoint(point=(3*Names.a1, -Names.a2,   0.0   ))
        a.ReferencePoint(point=(3*Names.a1,  0.0 ,       0.0  ))
        a.ReferencePoint(point=(3*Names.a1,  Names.a2,   0.0    ))
        a.ReferencePoint(point=(3*Names.a1, -Names.a2,  Names.a3   ))
        a.ReferencePoint(point=(3*Names.a1,  0.0 ,      Names.a3  ))
        a.ReferencePoint(point=(3*Names.a1,  Names.a2,  Names.a3   ))
    # First creating a set for the reference nodes
    applied_strain_keys = Names.APPLIED_STRAIN_KEYS 
    for i,k in enumerate(applied_strain_keys):
        a.Set(name='eps' + str(k), referencePoints=(a.referencePoints[a.features['RP-' + str(i+1)].id], ))

def createLoadCases(Model,Names):
    """
        Create 6 different load cases
            
        Inputs:
            Model - A model object from a Model Database
            Names - Class for global properties
    """
    for i in range(6):
        name = Names.LOAD_CASES_NAMES[i]
        bcs = [(name + '-eps' + str(ii), 1.0) for ii in Names.APPLIED_STRAIN_KEYS ]
        Model.steps[Names.STEP_NAME].LoadCase(name=name, 
                boundaryConditions=tuple(bcs))

def createBC_loadcases(Model,applied_strains,Names):
    """
        Create 54 boundary conditions for Column-1-eps11 ...
            
        Inputs:
            Model - A model object from a Model Database
            Names - Class for global properties
    """
    a = Model.rootAssembly
    # creating boundary conditions for the applied strain loading
    for load_case_name in Names.LOAD_CASES_NAMES: #[column_num:column_num+1]
        applied_strain = applied_strains[load_case_name]
        for i,k in enumerate(Names.APPLIED_STRAIN_KEYS ):
            # Writing displacement boundary condition for the values of the applied strain
            # U1, to write the CE's
            name = load_case_name+'-eps' + str(k)
            set_name = 'eps' + str(k)
            Model.DisplacementBC(name=name, 
                                        createStepName=Names.STEP_NAME, 
                                        region=a.sets[set_name], 
                                        u1=float(applied_strain[k]), 
                                        u2=const.UNSET, 
                                        u3=const.UNSET, 
                                        ur1=const.UNSET, 
                                        ur2=const.UNSET, 
                                        ur3=const.UNSET, 
                                        amplitude=const.UNSET, 
                                        fixed=const.OFF, 
                                        distributionType=const.UNIFORM, 
                                        fieldName='', localCsys=None)

def equation_constraint_create_faces(Model,master_set_name, slave_set_name,
                                     prefix_constraint ,
                                     coef_eps,eps_column):
    """
        Create equation constraint betweeen faces
            
        Inputs:
            Model - A model object from a Model Database
            master_set_name - The name of the master set name 
            slave_set_name  - The name of the slave set name
            prefix_constraint - Prefix that will be added in the definition of the constraint
            coef_eps - The coefficent that will appear for the DOF1, DOF2, and DOF3
            eps_column - The index of epsij where j is the eps_column, it can be 1,2,3
    """
    # Here terms are defined in tuples it can be any number but here we have defined 4 terms
    # This tuple is arranged as (coefficent, set_name, DOF)
    coef_master = -1.0
    coef_slave  = 1.0
    for dof in [1,2,3]:
        constraint_name = prefix_constraint + '_DOF_' + str(dof)   
        strain_name = 'eps' + str(dof) + str(eps_column) # epsir 
        Model.Equation(name=constraint_name, terms=(
                (coef_master, master_set_name, dof), 
                (coef_slave,  slave_set_name,  dof), 
                (coef_eps, strain_name, 1), 
                ))

def PBCsets(Model,Names):
    # creating basic sets
    createBaseSets(Model=Model,Names=Names)
    # creating pairing sorted sets
    pairsOfNodesetNames = [ (Names.POSITIVE_X_SET, Names.NEGATIVE_X_SET),
                            (Names.POSITIVE_Y_SUB_NEG_X, Names.NEGATIVE_Y_SUB_NEG_X),
                            (Names.POSITIVE_Z_SUB_NEG_X_SUB_NEG_Y, Names.NEGATIVE_Z_SUB_NEG_X_SUB_NEG_Y),]
    applyMechanicalPBC(Model=Model,pairsOfNodesetNames=pairsOfNodesetNames,Names=Names)


def pipline_3D_periodicity(Model,Names):
    # creating basic sets
    createBaseSets(Model=Model,Names=Names)
    # creating reference points
    create_referencePoints(Model=Model,Names=Names)
    # creating pairing sorted sets
    pairsOfNodesetNames = [ (Names.POSITIVE_X_SET, Names.NEGATIVE_X_SET),
                            (Names.POSITIVE_Y_SUB_NEG_X, Names.NEGATIVE_Y_SUB_NEG_X),
                            (Names.POSITIVE_Z_SUB_NEG_X_SUB_NEG_Y, Names.NEGATIVE_Z_SUB_NEG_X_SUB_NEG_Y),]
    applyMechanicalPBC(Model=Model,pairsOfNodesetNames=pairsOfNodesetNames,Names=Names)
    # updating base sets
    createBaseSets(Model=Model,Names=Names)
    # strain values for each loading case
    applied_strains = create_applied_loadings(Names)
    #Create boundary conditions    
    createBC_loadcases(Model=Model,applied_strains=applied_strains,Names=Names)
    
    # Creating constraints
    equation_constraint_create_faces(Model=Model,
                                    master_set_name= Names.NEGATIVE_X_SET + Names.SORTED_SET_SUFFIX
                                    ,slave_set_name= Names.POSITIVE_X_SET + Names.SORTED_SET_SUFFIX
                                    ,prefix_constraint = Names.PREFIX_CONSTRAINTS + '_X' ,coef_eps=-2*Names.a1,eps_column=1)
    equation_constraint_create_faces(Model=Model,
                                    master_set_name= Names.NEGATIVE_Y_SUB_NEG_X + Names.SORTED_SET_SUFFIX
                                    ,slave_set_name= Names.POSITIVE_Y_SUB_NEG_X + Names.SORTED_SET_SUFFIX
                                        ,prefix_constraint = Names.PREFIX_CONSTRAINTS + '_Y' ,coef_eps=-2*Names.a2,eps_column=2)
    equation_constraint_create_faces(Model=Model,
                                    master_set_name= Names.NEGATIVE_Z_SUB_NEG_X_SUB_NEG_Y + Names.SORTED_SET_SUFFIX
                                    ,slave_set_name= Names.POSITIVE_Z_SUB_NEG_X_SUB_NEG_Y + Names.SORTED_SET_SUFFIX
                                        ,prefix_constraint = Names.PREFIX_CONSTRAINTS + '_Z' ,coef_eps=-2*Names.a3,eps_column=3)
    # Creating load cases
    createLoadCases(Model=Model,Names=Names)
    
