import abaqus
import abaqusConstants as const
import mesh
import math
import sys
import numpy as np

def set_difference(list_nodes_set_1,list_nodes_set_2):
    '''
        It applies the set difference operation on node set 1 and node set 2
    '''
    set1 = {}; 
    set2={};
    for n in list_nodes_set_1:
        set1[n.label] = n
    for n in list_nodes_set_2:
        set2[n.label] = n
    common_nodes_keys = set(set1).difference(set(set2))
    nodes = []
    for k in common_nodes_keys:
        nodes.append(set1[k])
    return nodes

def nodes_sorting_faces(nodes,i,j,k):
    '''
        Sorts the array as per their coordinates, by i,j,k preference can be given from which coordinate it should
        be sorted first
        Inputs:
            nodes:             Can be group of nodes of right face, top face, bottom face, or left face
            i,j,k:             direction order i=0,j=1,k=1, means first sort by x axis then y axis and then z-axis
        Outputs:
            face_nodes_groups  It's a list of lists
    '''
    index_info = []
    for index,n in enumerate(nodes):
        x, y, z = n.coordinates
        index_info.append([x,y,z,index])
    sorted_points = sorted(index_info , key=lambda key: [key[i], key[j],key[k]])
    sorted_points = np.asarray(sorted_points)
    return mesh.MeshNodeArray([nodes[int(i)] for i in sorted_points[:,3]])


def create_applied_loadings(Names):
    applied_strains = {}
    load_cases_names = Names.LOAD_CASES_NAMES
    # For eps 11
    ones_except_0 = 1.0
    zeros = 0.0
    applied_strain = {11: ones_except_0,
                      12: zeros,
                      13: zeros,
                      21: zeros,
                      22: zeros,
                      23: zeros,
                      31: zeros,
                      32: zeros,
                      33: zeros,}
    applied_strains[load_cases_names[0]] = applied_strain
    # For eps 22
    applied_strain = {11: zeros,
                      12: zeros,
                      13: zeros,
                      21: zeros,
                      22: ones_except_0,
                      23: zeros,
                      31: zeros,
                      32: zeros,
                      33: zeros,}
    applied_strains[load_cases_names[1]] = applied_strain
    # For eps 33
    applied_strain = {11: zeros,
                      12: zeros,
                      13: zeros,
                      21: zeros,
                      22: zeros,
                      23: zeros,
                      31: zeros,
                      32: zeros,
                      33: ones_except_0,}
    applied_strains[load_cases_names[2]] = applied_strain
    # For eps 12
    ones_except_0 = 0.5
    applied_strain = {11: zeros,
                      12: ones_except_0,
                      13: zeros,
                      21: ones_except_0,
                      22: zeros,
                      23: zeros,
                      31: zeros,
                      32: zeros,
                      33: zeros,}
    applied_strains[load_cases_names[3]] = applied_strain
    # For eps 13
    applied_strain = {11: zeros,
                      12: zeros,
                      13: ones_except_0,
                      21: zeros,
                      22: zeros,
                      23: zeros,
                      31: ones_except_0,
                      32: zeros,
                      33: zeros,}
    applied_strains[load_cases_names[4]] = applied_strain
    # For eps 23
    applied_strain = {11: zeros,
                      12: zeros,
                      13: zeros,
                      21: zeros,
                      22: zeros,
                      23: ones_except_0,
                      31: zeros,
                      32: ones_except_0,
                      33: zeros,}
    applied_strains[load_cases_names[5]] = applied_strain
    return applied_strains

class UnmatchedNodeError(Exception):
    pass

def pairNodes(masterNodes, slaveNodes, vectorOfPeriodicity, relTolerance=1e-08):
    """
    Get arrays of master and slave nodes ordered so that they are paired for
    imposition of PBCs.
    
    This search is performed by looking for a match for each provided slave node
    If multiple nodes have the same coordinate, pairs are defined using the
    centroids of attached element facets.
    
    Raises UnmatchedNodeError if the sets cannot be paired, which can happen if
    the nodes are not positioned periodically or if there are multiple nodes at 
    a location and their attached facets do not match periodically.
    
    Inputs:
        masterNodes -         list containing master nodes
        slaveNodes -          list containing slave nodes
        vectorOfPeriodicity - vector running from master nodes to slave nodes
        relTolerance -        search tolerance relative to the diagonal of the
                              bounding box for one of the node sets
    Outputs:
        masterNodes_ordered, slaveNodes_ordered
    """
    
    slaveBounds = mesh.MeshNodeArray(list(slaveNodes)).getBoundingBox()
    absTolerance = relTolerance * math.sqrt(sum([ a - b for a, b in zip(slaveBounds['high'], slaveBounds['low']) ]))
    print('absolute search tolerance:%1.4e'%absTolerance)
    slaveNodes_ordered = mesh.MeshNodeArray(list(slaveNodes))
    coordsToSearch = [ tuple([ snc - pvc for snc, pvc in zip(sn.coordinates, vectorOfPeriodicity) ]) for sn in slaveNodes_ordered ]
    # print('coordsToSearch:',coordsToSearch)
    searchResults = mesh.MeshNodeArray(list(masterNodes)).getClosest(coordinates=coordsToSearch, searchTolerance=absTolerance, numToFind=10)
    if searchResults == None:
        error_msg = 'getClosest method returned None result'
        print(error_msg)
        raise RuntimeError(error_msg)
    foundDoubleNodes = False
    masterNodes_ordered = []
    for idx, (matchedMasters, slaveNode) in enumerate(zip(searchResults, slaveNodes_ordered)):
        if not matchedMasters:
            slaveNodeLabel = slaveNode.instanceName + '.' + str(slaveNode.label)
            message = 'Did not find master node matching slave ' + slaveNodeLabel + '\nwithin tolerance of coordinate ' + str([ a + b for a, b in zip(slaveNode.coordinates, vectorOfPeriodicity) ])
            message += '. Set index='+str(idx)
            print(message)
            raise UnmatchedNodeError(message)
        elif len(matchedMasters) > 1:
            foundDoubleNodes = True
            error_msg = 'Found double nodes check mesh'
            raise RuntimeError(error_msg)
        else:
            masterNodes_ordered.append(matchedMasters[0])
    if foundDoubleNodes == True:
        error_msg = 'Found double nodes check mesh'
        raise RuntimeError(error_msg)
    return (masterNodes_ordered, slaveNodes_ordered)


def getPeriodicityVector(MasterNodes, SlaveNodes):
    """
    Returns a list defining the periodicity vector that characterizes the pair of nodesets.
    This is based on the bounds of each set of nodes - the sets simply have to have the same
    extent.  They do not have to be periodically positioned
    
    Inputs:
    MasterNodes - The first collection of MeshNode references, representing the masters
    SlaveNodes -  The second collection of MeshNode references, representing the slaves
    
    Outputs:
    PeriodicityVector - a list of doubles defining the vector of periodicity between the
                        two nodesets.
    """
    masterNodesBounds = mesh.MeshNodeArray(list(MasterNodes)).getBoundingBox()
    slaveNodesBounds = mesh.MeshNodeArray(list(SlaveNodes)).getBoundingBox()
    return [ s - m for s, m in zip(slaveNodesBounds['low'], masterNodesBounds['low']) ]


def getAllElementList(Model):
    """
    Return a list of lists with all elements from all instances.
    """
    AllElements = [
     Model.rootAssembly.elements]
    for inst in Model.rootAssembly.instances.values():
        AllElements.append(inst.elements)
    return AllElements

def GetSurfaceFromNodeSet(inputNodes):
    """
    Return MeshFaceArray containing meshfaces that only touch nodes in InputNodes 
    
    Inputs:
    inputNodes -      Mesh node array
    
    Output:
    MeshFaceArray containing faces that have all nodes in inputNodes
    """
    nodesInSet = set([ (a.label, a.instanceName) for a in inputNodes ])
    FacesOnSurf = []
    FacesTouchingSurf = {}
    for n in inputNodes:
        for face in n.getElemFaces():
            if n.instanceName:
                FacesTouchingSurf[(face.label, face.face, n.instanceName)] = face
            else:
                FacesTouchingSurf[(face.label, face.face)] = face
    for f in FacesTouchingSurf.values():
        IsOnSurf = True
        for n in f.getNodes():
            if (
             n.label, n.instanceName) not in nodesInSet:
                IsOnSurf = False
                break
        
        if IsOnSurf == True:
            FacesOnSurf.append(f)
    
    MyMFA = mesh.MeshFaceArray(FacesOnSurf)
    return MyMFA

def getAllNodesList(Model):
    """
    Return a list of lists with all nodes from all instances.
    """
    AllNodes = [
     Model.rootAssembly.nodes]
    for inst in Model.rootAssembly.instances.values():
        AllNodes.append(inst.nodes)
    
    return AllNodes

def ComputeBoundingDimensionsFromMdb(mdbModel, getCoordinates=False):
    """
    ComputeBoundingDimensionsFromMdb:
    Computes the volume, dimensions and coordinates of the bounding box of all instances on the mdb model
    Supports multiple instances
    
    Inputs:
        mdbModel            -   Model object from the mdb that will be modificed
        getCoordinates      -   Optional: True to retrieve the bounding coordinates
    Outputs:
        modelVolume         -   volume of RVE including voids
        modelSize           -   list with model size [Lx,Ly,Lz]
      o modelCoordinates    -   list with tupples of bounding coordinates: [(minX,maxX),(minY,maxY),(minZ,maxZ)]
    """
    firstPass = True
    minCoord = [
     sys.float_info.max] * 3
    maxCoord = [-sys.float_info.max] * 3
    allNodes = getAllNodesList(mdbModel)
    for nodeArray in allNodes:
        if nodeArray:
            bounds = nodeArray.getBoundingBox()
            minCoord = [ min(this, old) for this, old in zip(bounds['low'], minCoord) ]
            maxCoord = [ max(this, old) for this, old in zip(bounds['high'], maxCoord) ]
    
    modelSize = [ abs(maxCoord[i] - minCoord[i]) for i in range(0, 3) ]
    modelVolume = modelSize[0] * modelSize[1] * modelSize[2]
    modelCoordinates = [ (minCoord[i], maxCoord[i]) for i in range(0, 3) ]
    if not getCoordinates:
        return (modelVolume, modelSize)
    else:
        return (
         modelVolume, modelSize, modelCoordinates)

def gatherNodesForAssemblySet(nodes):
    """
    Take list of nodes and put them in list of MeshNodeArray objects separated by instance
    This is how the set constructor for an assembly-level set wants them input.
    """
    nodesByInstance = {}
    for n in nodes:
        if n.instanceName:
            if n.instanceName in nodesByInstance:
                nodesByInstance[n.instanceName].append(n)
            else:
                nodesByInstance[n.instanceName] = [
                 n]
        elif 'ASSEMBLYNODES_asvjuinqwpiurbvqosfdhvoqrwegfophsv' in nodesByInstance:
            nodesByInstance['ASSEMBLYNODES_asvjuinqwpiurbvqosfdhvoqrwegfophsv'].append(n)
        else:
            nodesByInstance['ASSEMBLYNODES_asvjuinqwpiurbvqosfdhvoqrwegfophsv'] = [
             n]
    
    return [ mesh.MeshNodeArray(n) for n in nodesByInstance.values() ]

def GetNodeAtRVECenter(mdbModel, nodesToOmit=None):
    """
    GetNodeAtRVECenter:
    this function returns node object for the node which is closest to the center of RVE.
    
    Inputs:
    mdbModel-     A model object from a Model Database
    nodesToOmit - do not pick a node that is in this list of nodes
    """
    vol, dims, boundingCoords = ComputeBoundingDimensionsFromMdb(mdbModel, getCoordinates=True)
    CenterCoord = [ (c[1] + c[0]) / 2.0 for c in boundingCoords ]
    NodesToSearch = mdbModel.rootAssembly.Set(name='RVECenterNodeFinder_SearchNodes_8n48ncs8n', nodes=getAllNodesList(mdbModel))
    if nodesToOmit:
        OmitNodesTempSet = mdbModel.rootAssembly.Set(name='RVECenterNodeFinder_Omit_u9vnq3094m', nodes=gatherNodesForAssemblySet(nodesToOmit))
        NodesToSearch = mdbModel.rootAssembly.SetByBoolean(name='RVECenterNodeFinder_SearchNodes_8n48ncs8n', operation=const.DIFFERENCE, sets=(
            NodesToSearch, OmitNodesTempSet))
        del mdbModel.rootAssembly.sets['RVECenterNodeFinder_Omit_u9vnq3094m']
    ClosestNode = NodesToSearch.nodes.getClosest(coordinates=CenterCoord)
    del mdbModel.rootAssembly.sets['RVECenterNodeFinder_SearchNodes_8n48ncs8n']
    return ClosestNode

def copyFacets(Model, Facets, PartName, translateVector=None):
    """
    Copy provided facets to elements on a new part
    
    Inputs:
    Outputs:
        A MeshElementArray of the created elements.
    """
    ElTypeMap = {3: const.TRI3, 4: const.QUAD4, 
       6: const.TRI6, 
       8: const.QUAD8}
    part = Model.Part(name=PartName)
    CopiedEls = []
    newNodes = {}
    for facet in Facets:
        nodes = facet.getNodes()
        for n in nodes:
            if (
             n.instanceName, n.label) not in newNodes:
                coords = n.coordinates if translateVector == None else tuple([ a + b for a, b in zip(n.coordinates, translateVector) ])
                newNodes[(n.instanceName, n.label)] = part.Node(coordinates=coords)
        
        newNodesForElement = [ newNodes[(n.instanceName, n.label)] for n in nodes ]
        CopiedEls.append(part.Element(nodes=newNodesForElement, elemShape=ElTypeMap[len(nodes)]))
    
    inst = Model.rootAssembly.Instance(name=PartName, part=part, dependent=const.ON)
    return inst.elements[:]

def copyMeshFacets(modelObject, nodeSetNameToCopy, translateVector,Names):
    """
    Copies the element facets of nodesToCopy and moves them by translateVector.
    
    Inputs:
        modelObject -     The mdb model object
        nodesToCopy -     Name of assembly-level nodeset defining facets to be copied
        translateVector - Vector going from nodesToCopy to nodesToTie
    Outputs:
        Name of assembly-level set containing the created elements
    """
    FacetsToCopy = GetSurfaceFromNodeSet(modelObject.rootAssembly.sets[nodeSetNameToCopy].nodes)
    NewElements  = copyFacets(modelObject, FacetsToCopy, nodeSetNameToCopy + Names.COPIED_FACETS_SUFFIX, translateVector)
    createdElsElset = modelObject.rootAssembly.Set(name=nodeSetNameToCopy + Names.COPIED_FACETS_SUFFIX, elements=NewElements)
    return nodeSetNameToCopy + Names.COPIED_FACETS_SUFFIX

def getPairsOfMatchedNodes(modelObject, nodePairs,Names):
    """
    Pair up nodes for imposition of periodic constraints.
    
    Inputs:
        modelObject -   The mdb model object
        nodePairs -     List of pairs (list) of assembly-level nodeset names to be constrained.
                        The first item in each pair will be master, second will be slave

    Outputs:
        (pairsForConstraints, vectorsOfPeriodicity, createdSurfaceElements, nodesToIgnore)
        pairsForConstraints -      a list of pairs (list) of mesh node arrays that are ordered
                                   so that nodes are appropriately paired
        vectorsOfPeriodicity -     A list of tuples giving the vector of periodicity for 
                                   each pair of node arrays.

    """
    AllCreatedSurfElements = []
    modelObject.rootAssembly.Set(name=Names.ALL_ORIGINAL_ELEMENTS_SET, elements=getAllElementList(modelObject))
    ignoreNodeset = None
    pairsForConstraints = []
    vectorsOfPeriodicity = []
    createdSurfaceElementObjects = []
    for masterName, slaveName in nodePairs:
        masterSet = modelObject.rootAssembly.sets[masterName]
        slaveSet  = modelObject.rootAssembly.sets[slaveName]
        thisPeriodicVec = getPeriodicityVector(masterSet.nodes, slaveSet.nodes)
        reducedSlaveNodeSet = slaveSet 
        print('Pairing ' + masterName + ' and ' + slaveName)
        print('Periodicity Vector=',thisPeriodicVec)
        periodicPairedNodes = None
        try:
            periodicPairedNodes = pairNodes(masterSet.nodes, 
                                            reducedSlaveNodeSet.nodes, 
                                            thisPeriodicVec, 
                                            Names.RELATIVE_PERIODIC_POSITION_TOLERANCE)
        except UnmatchedNodeError:
            print('Meshes are not periodic within tolerance - using non-periodic facet-copying')
            copiedFacetSetName = copyMeshFacets(modelObject, slaveName, [ -a for a in thisPeriodicVec ],Names)
            copiedFacetSet = modelObject.rootAssembly.sets[copiedFacetSetName]
            copiedFacetSurface = modelObject.rootAssembly.Surface(name=copiedFacetSetName, side1Elements=copiedFacetSet.elements)
            masterFacets = GetSurfaceFromNodeSet(masterSet.nodes)
            masterFacetSurface = modelObject.rootAssembly.Surface(name=masterName, face1Elements=masterFacets)
            modelObject.Tie(name=Names.TIE_PREFIX + copiedFacetSetName + '_to_' + masterName, master=masterFacetSurface, slave=copiedFacetSurface, adjust=const.OFF, constraintEnforcement=const.SURFACE_TO_SURFACE, thickness=const.OFF)
            try:
                periodicPairedNodes = pairNodes(copiedFacetSet.nodes, reducedSlaveNodeSet.nodes, thisPeriodicVec, Names.RELATIVE_PERIODIC_POSITION_TOLERANCE)
            except UnmatchedNodeError as detail:
                print ('Unable to find master node in ' + copiedFacetSetName + ' for slave in ' + slaveName + Names.REDUCED_SET_SUFFIX + '\n', detail)
                raise RuntimeError
            createdSurfaceElementObjects.append(copiedFacetSetName)
        else:
            print('Meshes are periodic within tolerance - pairing nodes directly')
            createdSurfaceElementObjects.append(None)
        pairsForConstraints.append(periodicPairedNodes)
        if ignoreNodeset:
            ignoreNodeset = modelObject.rootAssembly.SetByBoolean(name=Names.AUTOGENERATED_PREFIX + 'NodePairing_IgnoreNodeset', operation=const.UNION, sets=[
             ignoreNodeset, reducedSlaveNodeSet])
        else:
            ignoreNodeset = modelObject.rootAssembly.Set(name=Names.AUTOGENERATED_PREFIX + 'NodePairing_IgnoreNodeset', objectToCopy=reducedSlaveNodeSet)
        vectorsOfPeriodicity.append(thisPeriodicVec)
    if ignoreNodeset is not None:
        nodesToIgnore = list(ignoreNodeset.nodes)
    else:
        nodesToIgnore = []
    # del modelObject.rootAssembly.sets[Names.AUTOGENERATED_PREFIX + 'NodePairing_IgnoreNodeset']    
    return (pairsForConstraints, vectorsOfPeriodicity, createdSurfaceElementObjects, nodesToIgnore)


def pinNodeSet(Model, nodesetToPin):
    """
    Pins a nodeset from the beginning of the analysis.
    
    Inputs:
    Model - A model object from a Model Database
    
    nodesetToPin -  node set name as a string   
    """
    stepNames = Model.steps.keys()
    regionPin = Model.rootAssembly.sets[nodesetToPin]
    Model.PinnedBC(createStepName='%s' % stepNames[0], name='PinnedNode', region=regionPin)

def CreateReferencePoints(mdbModel, referencePointNames):
    """
    CreateReferencePoints:
    Creates Reference Points for prescribing pbc and loads for a coupled temperature 
    displacement analysis.
    
    Inputs:
        mdbModel            -   abaqus.mdb.model object referring to the desired RVE model
        referencePointNames -   list of names of the reference points
    Outputs:
        [RPs]               -   list of set objects containing each a reference point:
                                    [setForUniaxial, setForShear, setForTemperature]
    """
    setList = []
    for nodeName in referencePointNames:
        eachNode = mdbModel.rootAssembly.Node((0, 0, 0))
        setList.append(mdbModel.rootAssembly.Set(nodeName, mesh.MeshNodeArray([eachNode])))
    return setList

def applyMechanicalPBC(Model,pairsOfNodesetNames,Names):
    """
    Applies periodic boundary conditions to an RVE
        
    Inputs:
    Model - A model object from a Model Database
    """
    userDefinedNodesetPairs = []
    originallySlavedNodes   = []
    previouslySlavedNodes   = []
    pairedNodes, periodicityVecs, createdSurfaceElements, previouslySlavedNodes = getPairsOfMatchedNodes(Model, pairsOfNodesetNames,Names)
    pairsOfSortedNodesetNames = []
    constraintKeywordString = ''
    SurfElementTypesLinear = (
     mesh.ElemType(elemCode=const.SFM3D3, elemLibrary=const.STANDARD),
     mesh.ElemType(elemCode=const.SFM3D4, elemLibrary=const.STANDARD))
    SurfElementTypesQuadratic = (mesh.ElemType(elemCode=const.SFM3D6, elemLibrary=const.STANDARD),
     mesh.ElemType(elemCode=const.SFM3D8, elemLibrary=const.STANDARD))
    SurfSec = Model.SurfaceSection(name=Names.SURFACE_ELEMENT_SECTION_NAME) if any(createdSurfaceElements) else None
    AllSurfElements = []
    for instanceSurfElements in createdSurfaceElements:
        if instanceSurfElements:
            surfElAssemblySet = Model.rootAssembly.sets[instanceSurfElements]
            AllSurfElements.append(surfElAssemblySet.elements[:])
            instanceName = surfElAssemblySet.elements[0].instanceName
            instance = Model.rootAssembly.instances[instanceName]
            part = instance.part
            AllPartElSet = part.Set(name=instanceSurfElements, elements=part.elements)
            AllPartLinearElements = [ e for e in AllPartElSet.elements if len(e.getNodes()) == 3 or len(e.getNodes()) == 4 ]
            if AllPartLinearElements:
                AllPartLinElSet = part.Set(name=instanceSurfElements + '_Linear', elements=mesh.MeshElementArray(AllPartLinearElements))
                part.setElementType(regions=AllPartLinElSet, elemTypes=SurfElementTypesLinear)
                del part.sets[instanceSurfElements + '_Linear']
            AllPartQuadraticElements = [ e for e in AllPartElSet.elements if len(e.getNodes()) == 6 or len(e.getNodes()) == 8 ]
            if AllPartQuadraticElements:
                AllPartQuadElSet = part.Set(name=instanceSurfElements + '_Quadratic', elements=mesh.MeshElementArray(AllPartQuadraticElements))
                part.setElementType(regions=AllPartQuadElSet, elemTypes=SurfElementTypesQuadratic)
                del part.sets[instanceSurfElements + '_Quadratic']
            part.SectionAssignment(region=AllPartElSet, sectionName=SurfSec.name)
    
    if AllSurfElements:
        AllSurfElSet = Model.rootAssembly.Set(name=Names.SURFACE_ELEMENTS_SET, elements=AllSurfElements)
    
    PinVertexSet = Model.rootAssembly.Set(name=Names.MECHANICAL_PIN_VERTEX, nodes=mesh.MeshNodeArray([GetNodeAtRVECenter(Model, nodesToOmit=previouslySlavedNodes)]))
    RPSets = CreateReferencePoints(Model, (Names.MECHANICAL_NORM_REF_NODE, Names.MECHANICAL_SHEAR_REF_NODE))
    
    pinNodeSet(Model, Names.MECHANICAL_PIN_VERTEX)
    a = abaqus.mdb.models[Names.MODEL_NAME].rootAssembly
    # print('Before for loop')
    # i=0
    # print(pairsOfNodesetNames)
    # print('*'*50)
    # print(pairedNodes)
    for nodeSetPairNames, pairOfSortedNodeLists in zip(pairsOfNodesetNames, pairedNodes):
            # i+=1
            # print('Loop i=i'%i)
            pairsOfSortedNodesetNames.append([ n + Names.SORTED_SET_SUFFIX for n in nodeSetPairNames ])
            # print(pairsOfSortedNodesetNames)
            for sortedNodesetName, sortedNodes in zip(pairsOfSortedNodesetNames[-1], pairOfSortedNodeLists):
                set_nodes = []
                for n in sortedNodes:
                    set_nodes.append(n.label)
                a.SetFromNodeLabels(name=str(sortedNodesetName), nodeLabels=((Names.INSTANCE_NAME, tuple(set_nodes)), ), unsorted=True)

# applyMechanicalPBC(Model=createdModel,pairsOfNodesetNames=pairsOfNodesetNames,Names=Names)