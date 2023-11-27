import math, mesh, abaqus, regionToolset
from abaqusConstants import *

def generateHexFiberArray(vf, fiber_radius, interface_ratio, depth, n_rad, n_depth, RVE_modelName,
                          matPropties):
    mdb = abaqus.mdb
    mdb.Model(name=RVE_modelName)
    HexPack = mdb.models[RVE_modelName]
    vf = float(vf)
    fiber_radius = float(fiber_radius)
    interface_ratio = float(interface_ratio)
    depth = float(depth)
    n_rad = int(n_rad)
    n_depth = int(n_depth)
    print('n_rad = %i' % n_rad)
    print('n_depth = %i' % n_depth)
    s_hex = math.sqrt(2 * math.sqrt(3) * math.pi * pow(fiber_radius, 2) / (9 * vf))
    d_min = s_hex * math.sqrt(3)
    d_maj = 3 * s_hex
    if interface_ratio:
        t_int = interface_ratio * fiber_radius
    else:
        dv = s_hex - fiber_radius
        ds = 0.5 * math.sqrt(3) * s_hex - fiber_radius
        l1 = ds / (n_rad if n_rad > 1 else n_rad + 1)
        t_int = l1
    if vf * pow(1 + t_int / fiber_radius, 2) > math.sqrt(3) * math.pi / 6:
        print ('Combination of vf and thickness interface ratio too large')
        raise RuntimeError('Exceeded circle close packing')
    h = d_maj
    w = d_min
    radius = fiber_radius
    offset = t_int
    HexPackSketch = HexPack.ConstrainedSketch(name='HexPackSketch', sheetSize=10.0)
    HexPackSketch.rectangle(point1=(0, 0), point2=(depth, w / 2))
    HexPackPart = HexPack.Part(name='HexPackPart', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    HexPackPart.BaseSolidExtrude(depth=h / 2, sketch=HexPack.sketches['HexPackSketch'])
    HexPackPart.DatumPlaneByThreePoints(point1=(0.0, 0.0, 0.0), point2=(depth, 0.0, 0.0), point3=(0.0, w / 2, 0.0))
    HexPackPart.features.changeKey(fromName='Datum plane-1', toName='d1')
    HexPackPart.DatumPlaneByThreePoints(point1=(0.0, 0.0, 0.0), point2=(depth, 0.0, 0.0), point3=(0.0, 0.0, h / 2))
    HexPackPart.features.changeKey(fromName='Datum plane-1', toName='d2')
    HexPackPart.DatumPlaneByThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, 0.0), point3=(0.0, 0.0, h / 2))
    HexPackPart.features.changeKey(fromName='Datum plane-1', toName='d3')
    ed1 = HexPackPart.edges
    edgeAB = ed1.findAt((0.0, w / 4, 0.0))
    edgeACindex = edgeAB.index
    HexPackPart.Set(edges=ed1[edgeACindex:edgeACindex + 1], name='EdgeAB')
    fc1 = HexPackPart.faces
    face1 = fc1.findAt((0.0, h / 4, w / 4))
    faceAindex = face1.index
    f = HexPackPart.faces
    e1, d1 = (HexPackPart.edges, HexPackPart.datums)
    t = HexPackPart.MakeSketchTransform(sketchPlane=f[face1.index], sketchUpEdge=e1[edgeAB.index], sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
    PartitionSketch = HexPack.ConstrainedSketch(name='PartitionSketch', sheetSize=10.0, transform=t)
    PartitionSketch.setPrimaryObject(option=SUPERIMPOSE)
    HexPackPart.projectReferencesOntoSketch(sketch=PartitionSketch, filter=COPLANAR_EDGES)
    PartitionSketch.ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, radius), point2=(radius, 0.0), direction=CLOCKWISE)
    PartitionSketch.ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, radius + offset), point2=(radius + offset, 0.0), direction=CLOCKWISE)
    PartitionSketch.ArcByCenterEnds(center=(h / 2, w / 2), point1=(h / 2 - radius, w / 2), point2=(h / 2, w / 2 - radius), direction=COUNTERCLOCKWISE)
    PartitionSketch.ArcByCenterEnds(center=(h / 2, w / 2), point1=(h / 2 - (radius + offset), w / 2), point2=(h / 2, w / 2 - (radius + offset)), direction=COUNTERCLOCKWISE)
    HexPackPart.ShellExtrude(sketchPlane=f[face1.index], sketchUpEdge=e1[edgeAB.index], sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=PartitionSketch, depth=depth, flipExtrudeDirection=ON, keepInternalBoundaries=ON)
    PartitionSketch.unsetPrimaryObject()
    ed1 = HexPackPart.edges
    edgeAB = ed1.findAt((0.0, w / 4, 0.0))
    edgeACindex = edgeAB.index
    HexPackPart.Set(edges=ed1[edgeACindex:edgeACindex + 1], name='EdgeAB')
    fc1 = HexPackPart.faces
    face1 = fc1.findAt((0.0, h / 4, w / 4))
    faceAindex = face1.index
    f = HexPackPart.faces
    e1, d1 = (HexPackPart.edges, HexPackPart.datums)
    t = HexPackPart.MakeSketchTransform(sketchPlane=f[face1.index], sketchUpEdge=e1[edgeAB.index], sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
    hexPartitionSketch = HexPack.ConstrainedSketch(name='hexPartitionSketch', sheetSize=10.0, transform=t)
    hexPartitionSketch.setPrimaryObject(option=SUPERIMPOSE)
    HexPackPart.projectReferencesOntoSketch(sketch=hexPartitionSketch, filter=COPLANAR_EDGES)
    hexPartitionSketch.Line(point1=(0.0, math.sqrt(3) / 2 * (2 * radius / 3)), point2=(radius / 3, math.sqrt(3) / 2 * (2 * radius / 3)))
    hexPartitionSketch.Line(point1=(radius / 3, math.sqrt(3) / 2 * (2 * radius / 3)), point2=(2 * radius / 3, 0.0))
    hexPartitionSketch.Line(point1=(h / 2, w / 2 - math.sqrt(3) / 2 * (2 * radius / 3)), point2=(h / 2 - radius / 3, w / 2 - math.sqrt(3) / 2 * (2 * radius / 3)))
    hexPartitionSketch.Line(point1=(h / 2 - radius / 3, w / 2 - math.sqrt(3) / 2 * (2 * radius / 3)), point2=(h / 2 - 2 * radius / 3, w / 2))
    HexPackPart.ShellExtrude(sketchPlane=f[face1.index], sketchUpEdge=e1[edgeAB.index], sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=hexPartitionSketch, depth=depth, flipExtrudeDirection=ON, keepInternalBoundaries=ON)
    hexPartitionSketch.unsetPrimaryObject()
    ed1 = HexPackPart.edges
    edgeAB = ed1.findAt((0.0, w / 4, 0.0))
    edgeACindex = edgeAB.index
    HexPackPart.Set(edges=ed1[edgeACindex:edgeACindex + 1], name='EdgeAB')
    fc1 = HexPackPart.faces
    face1 = fc1.findAt((0.0, h / 4, w / 4))
    faceAindex = face1.index
    f = HexPackPart.faces
    e1, d1 = (HexPackPart.edges, HexPackPart.datums)
    t = HexPackPart.MakeSketchTransform(sketchPlane=f[face1.index], sketchUpEdge=e1[edgeAB.index], sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
    hexcellPartitionSketch = HexPack.ConstrainedSketch(name='hexcellPartitionSketch', sheetSize=10.0, transform=t)
    hexcellPartitionSketch.setPrimaryObject(option=SUPERIMPOSE)
    HexPackPart.projectReferencesOntoSketch(sketch=hexcellPartitionSketch, filter=COPLANAR_EDGES)
    hexcellPartitionSketch.Line(point1=(h / 2 - s_hex / 2, 0.0), point2=(s_hex / 2, w / 2))
    hexcellPartitionSketch.Line(point1=(radius / 3, math.sqrt(3) / 2 * (2 * radius / 3)), point2=(s_hex / 2, w / 2))
    hexcellPartitionSketch.Line(point1=(h / 2 - radius / 3, w / 2 - math.sqrt(3) / 2 * (2 * radius / 3)), point2=(h / 2 - s_hex / 2, 0.0))
    HexPackPart.ShellExtrude(sketchPlane=f[face1.index], sketchUpEdge=e1[edgeAB.index], sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=hexcellPartitionSketch, depth=depth, flipExtrudeDirection=ON, keepInternalBoundaries=ON)
    hexcellPartitionSketch.unsetPrimaryObject()
    c1 = HexPackPart.cells
    cellA = c1.findAt((0.0, 0.0, radius + offset + offset))
    cellAindex = cellA.index
    HexPackPart.Set(cells=c1[cellAindex:cellAindex + 1], name='middle_cell_1')
    HexPackPart.PartitionCellByPlaneThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, h / 2), point3=(depth, 0.0, 0.0), cells=c1[cellA.index])
    c4 = HexPackPart.cells
    cellA = c4.findAt((0.0, w / 2, h / 2 - (radius + offset + offset)))
    cellAindex = cellA.index
    HexPackPart.Set(cells=c4[cellAindex:cellAindex + 1], name='middle_cell_2')
    HexPackPart.PartitionCellByPlaneThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, h / 2), point3=(depth, 0.0, 0.0), cells=c4[cellA.index])
    c2 = HexPackPart.cells
    cellB = c2.findAt((0.0, 0.0, radius + offset / 2))
    cellBindex = cellB.index
    HexPackPart.Set(cells=c2[cellBindex:cellBindex + 1], name='cohesive_zoneA_cell')
    HexPackPart.PartitionCellByPlaneThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, h / 2), point3=(depth, 0.0, 0.0), cells=c2[cellB.index])
    c3 = HexPackPart.cells
    cellC = c3.findAt((0.0, w / 2, h / 2 - (radius + offset / 2)))
    cellCindex = cellC.index
    HexPackPart.Set(cells=c3[cellCindex:cellCindex + 1], name='cohesive_zoneB_cell')
    HexPackPart.PartitionCellByPlaneThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, h / 2), point3=(depth, 0.0, 0.0), cells=c3[cellC.index])
    c4 = HexPackPart.cells
    cellD = c4.findAt((0.0, 0.0, radius - offset / 2))
    cellDindex = cellD.index
    HexPackPart.Set(cells=c4[cellDindex:cellDindex + 1], name='fiber_cell_A')
    HexPackPart.PartitionCellByPlaneThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, h / 2), point3=(depth, 0.0, 0.0), cells=c4[cellD.index])
    c3 = HexPackPart.cells
    cellC = c3.findAt((0.0, w / 2, h / 2 - (radius - offset / 2)))
    cellCindex = cellC.index
    HexPackPart.Set(cells=c3[cellCindex:cellCindex + 1], name='cohesive_zoneB_cell')
    HexPackPart.PartitionCellByPlaneThreePoints(point1=(0.0, 0.0, 0.0), point2=(0.0, w / 2, h / 2), point3=(depth, 0.0, 0.0), cells=c3[cellC.index])
    fc4 = HexPackPart.faces
    face4 = fc4.findAt((depth / 2, 0.0, h / 4))
    faceAindex = face4.index
    HexPackPart.Mirror(mirrorPlane=fc4[face4.index], keepOriginal=ON, keepInternalBoundaries=ON)
    fc5 = HexPackPart.faces
    face5 = fc5.findAt((depth / 2, w / 4, 0.0))
    faceAindex = face5.index
    HexPackPart.Mirror(mirrorPlane=fc5[face5.index], keepOriginal=ON, keepInternalBoundaries=ON)
    c1 = HexPackPart.cells
    S = radius + offset / 2
    cellA = c1.getByBoundingCylinder((0, 0.0, 0.0), (depth * 100, 0.0, 0.0), S)
    HexPackPart.Set(cells=cellA, name='center_fiber')
    c1 = HexPackPart.cells
    S = radius + 1.1 * offset
    cellA = c1.getByBoundingCylinder((0, 0.0, 0.0), (depth * 100, 0.0, 0.0), S)
    HexPackPart.Set(cells=cellA, name='center_fiber_coh_zone')
    HexPackPart.SetByBoolean(name='coh_center_ring', operation=DIFFERENCE, sets=(
     HexPackPart.sets['center_fiber_coh_zone'], HexPackPart.sets['center_fiber']))
    c1 = HexPackPart.cells
    S = radius + offset / 2
    cellA = c1.getByBoundingCylinder((0, w / 2, h / 2), (depth * 100, w / 2, h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner1_fiber')
    c1 = HexPackPart.cells
    S = radius + 1.1 * offset
    cellA = c1.getByBoundingCylinder((0, w / 2, h / 2), (depth * 100, w / 2, h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner1_fiber_coh_zone')
    HexPackPart.SetByBoolean(name='coh_corner1_ring', operation=DIFFERENCE, sets=(
     HexPackPart.sets['corner1_fiber_coh_zone'], HexPackPart.sets['corner1_fiber']))
    c1 = HexPackPart.cells
    S = radius + offset / 2
    cellA = c1.getByBoundingCylinder((0, -w / 2, -h / 2), (depth * 100, -w / 2, -h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner2_fiber')
    c1 = HexPackPart.cells
    S = radius + 1.1 * offset
    cellA = c1.getByBoundingCylinder((0, -w / 2, -h / 2), (depth * 100, -w / 2, -h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner2_fiber_coh_zone')
    HexPackPart.SetByBoolean(name='coh_corner2_ring', operation=DIFFERENCE, sets=(
     HexPackPart.sets['corner2_fiber_coh_zone'], HexPackPart.sets['corner2_fiber']))
    c1 = HexPackPart.cells
    S = radius + offset / 2
    cellA = c1.getByBoundingCylinder((0, w / 2, -h / 2), (depth * 100, w / 2, -h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner3_fiber')
    c1 = HexPackPart.cells
    S = radius + 1.1 * offset
    cellA = c1.getByBoundingCylinder((0, w / 2, -h / 2), (depth * 100, w / 2, -h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner3_fiber_coh_zone')
    HexPackPart.SetByBoolean(name='coh_corner3_ring', operation=DIFFERENCE, sets=(
     HexPackPart.sets['corner3_fiber_coh_zone'], HexPackPart.sets['corner3_fiber']))
    c1 = HexPackPart.cells
    S = radius + offset / 2
    cellA = c1.getByBoundingCylinder((0, -w / 2, h / 2), (depth * 100, -w / 2, h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner4_fiber')
    c1 = HexPackPart.cells
    S = radius + 1.1 * offset
    cellA = c1.getByBoundingCylinder((0, -w / 2, h / 2), (depth * 100, -w / 2, h / 2), S)
    HexPackPart.Set(cells=cellA, name='corner4_fiber_coh_zone')
    HexPackPart.SetByBoolean(name='coh_corner4_ring', operation=DIFFERENCE, sets=(
     HexPackPart.sets['corner4_fiber_coh_zone'], HexPackPart.sets['corner4_fiber']))
    HexPackPart.SetByBoolean(name='cohesive_zone', sets=(HexPackPart.sets['coh_corner1_ring'],
     HexPackPart.sets['coh_corner2_ring'], HexPackPart.sets['coh_corner3_ring'],
     HexPackPart.sets['coh_corner4_ring'], HexPackPart.sets['coh_center_ring']))
    HexPackPart.SetByBoolean(name='all_fiber_zone', sets=(HexPackPart.sets['corner4_fiber'],
     HexPackPart.sets['corner3_fiber'], HexPackPart.sets['corner2_fiber'], HexPackPart.sets['corner1_fiber'],
     HexPackPart.sets['center_fiber']))
    HexPackPart.SetByBoolean(name='all_fiber_coh_zone', sets=(HexPackPart.sets['all_fiber_zone'],
     HexPackPart.sets['cohesive_zone']))
    c1 = HexPackPart.cells
    HexPackPart.Set(cells=c1, name='all_cells')
    if interface_ratio == 0:
        HexPackPart.SetByBoolean(name='matrix_zone', operation=DIFFERENCE, sets=(
         HexPackPart.sets['all_cells'], HexPackPart.sets['all_fiber_zone']))
    else:
        HexPackPart.SetByBoolean(name='matrix_zone', operation=DIFFERENCE, sets=(
         HexPackPart.sets['all_cells'], HexPackPart.sets['all_fiber_coh_zone']))
    HexPack.Material(name='MatFiber')
    HexPack.materials['MatFiber'].Elastic(table=((matPropties['fiber'].E, matPropties['fiber'].nu),))
    HexPack.Material(name='MatMatrix')
    HexPack.materials['MatMatrix'].Elastic(table=((matPropties['matrix'].E, matPropties['matrix'].nu),))
    if not interface_ratio == 0:
        HexPack.Material(name='MatInterface')
        HexPack.materials['MatInterface'].Elastic(table=(
         (3350000000.0,
          0.35),))
    HexPack.HomogeneousSolidSection(name='fiber', material='MatFiber', thickness=None)
    HexPack.HomogeneousSolidSection(name='matrix', material='MatMatrix', thickness=None)
    if not interface_ratio == 0:
        HexPack.HomogeneousSolidSection(name='interface', material='MatInterface', thickness=None)
    region = HexPackPart.sets['all_fiber_zone']
    HexPackPart.SectionAssignment(region=region, sectionName='fiber', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    region = HexPackPart.sets['cohesive_zone']
    if not interface_ratio == 0:
        HexPackPart.SectionAssignment(region=region, sectionName='interface', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    region = HexPackPart.sets['matrix_zone']
    HexPackPart.SectionAssignment(region=region, sectionName='matrix', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    d = depth
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((0.0, radius * math.sin(15 * math.pi / 180), radius * math.cos(15 * math.pi / 180)))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((0.0, radius * math.sin(45 * math.pi / 180), radius * math.cos(45 * math.pi / 180)))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((0.0, radius * math.sin(75 * math.pi / 180), radius * math.cos(75 * math.pi / 180)))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((0.0, (radius + offset) * math.sin(15 * math.pi / 180), (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((0.0, (radius + offset) * math.sin(45 * math.pi / 180), (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((0.0, (radius + offset) * math.sin(75 * math.pi / 180), (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((0.0, w / 2 - radius * math.sin(15 * math.pi / 180), h / 2 - radius * math.cos(15 * math.pi / 180)))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((0.0, w / 2 - radius * math.sin(45 * math.pi / 180), h / 2 - radius * math.cos(45 * math.pi / 180)))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((0.0, w / 2 - radius * math.sin(75 * math.pi / 180), h / 2 - radius * math.cos(75 * math.pi / 180)))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((0.0, w / 2 - (radius + offset) * math.sin(15 * math.pi / 180), h / 2 - (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((0.0, w / 2 - (radius + offset) * math.sin(45 * math.pi / 180), h / 2 - (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((0.0, w / 2 - (radius + offset) * math.sin(75 * math.pi / 180), h / 2 - (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((0.0, 0.75 / 2 * w, 1.25 / 2 * s_hex))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((0.0, w / 8, 1.75 / 2 * s_hex))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((0.0, 0.0, radius + offset / 2))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((0.0, (radius + offset / 2) * math.sin(30 * math.pi / 180), (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((0.0, (radius + offset / 2) * math.sin(60 * math.pi / 180), (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((0.0, radius + offset / 2, 0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - (radius + offset / 2), h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2, h / 2 - (radius + offset / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180), h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180), h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, w / 2 - yy * math.sin(30 * math.pi / 180), h / 2 - yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, yy * math.sin(30 * math.pi / 180), yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2, s_hex / 4))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2, LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, GG, 0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, 0.0, s_hex - (s_hex - (radius + offset)) * 0.5))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, 0.0, 1.25 * s_hex))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, uu * math.cos(30 * math.pi / 180), uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - uu * math.cos(30 * math.pi / 180), h / 2 - uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, (w / 2 - (radius + offset)) * 0.5, h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((0.0, radius * math.sin(15 * math.pi / 180), -(radius * math.cos(15 * math.pi / 180))))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH2')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((0.0, radius * math.sin(45 * math.pi / 180), -(radius * math.cos(45 * math.pi / 180))))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI2')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((0.0, radius * math.sin(75 * math.pi / 180), -(radius * math.cos(75 * math.pi / 180))))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU2')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((0.0, (radius + offset) * math.sin(15 * math.pi / 180), -((radius + offset) * math.cos(15 * math.pi / 180))))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK2')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((0.0, (radius + offset) * math.sin(45 * math.pi / 180), -((radius + offset) * math.cos(45 * math.pi / 180))))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL2')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((0.0, (radius + offset) * math.sin(75 * math.pi / 180), -((radius + offset) * math.cos(75 * math.pi / 180))))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT2')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((0.0, w / 2 - radius * math.sin(15 * math.pi / 180), -(h / 2 - radius * math.cos(15 * math.pi / 180))))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB2')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((0.0, w / 2 - radius * math.sin(45 * math.pi / 180), -(h / 2 - radius * math.cos(45 * math.pi / 180))))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC2')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((0.0, w / 2 - radius * math.sin(75 * math.pi / 180), -(h / 2 - radius * math.cos(75 * math.pi / 180))))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR2')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((0.0, w / 2 - (radius + offset) * math.sin(15 * math.pi / 180), -(h / 2 - (radius + offset) * math.cos(15 * math.pi / 180))))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE2')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((0.0, w / 2 - (radius + offset) * math.sin(45 * math.pi / 180), -(h / 2 - (radius + offset) * math.cos(45 * math.pi / 180))))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF2')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((0.0, w / 2 - (radius + offset) * math.sin(75 * math.pi / 180), -(h / 2 - (radius + offset) * math.cos(75 * math.pi / 180))))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS2')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((0.0, 0.75 / 2 * w, -(1.25 / 2 * s_hex)))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO2')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((0.0, w / 8, -(1.75 / 2 * s_hex)))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN2')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((0.0, 0.0, -(radius + offset / 2)))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV2')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((0.0, (radius + offset / 2) * math.sin(30 * math.pi / 180), -((radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW2')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((0.0, (radius + offset / 2) * math.sin(60 * math.pi / 180), -((radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX2')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((0.0, radius + offset / 2, -0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - (radius + offset / 2), -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2, -(h / 2 - (radius + offset / 2))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180), -(h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180), -(h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON2')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, w / 2 - yy * math.sin(30 * math.pi / 180), -(h / 2 - yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM2')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, yy * math.sin(30 * math.pi / 180), -(yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2, -(s_hex / 4)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR2')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2, -LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM2')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, GG, -0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, 0.0, -(s_hex - (s_hex - (radius + offset)) * 0.5)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, 0.0, -(1.25 * s_hex)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC2')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, uu * math.cos(30 * math.pi / 180), -(uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, w / 2 - uu * math.cos(30 * math.pi / 180), -(h / 2 - uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG2')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, (w / 2 - (radius + offset)) * 0.5, -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG2')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((0.0, -(radius * math.sin(15 * math.pi / 180)), -(radius * math.cos(15 * math.pi / 180))))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH4')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((0.0, -(radius * math.sin(45 * math.pi / 180)), -(radius * math.cos(45 * math.pi / 180))))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI4')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((0.0, -(radius * math.sin(75 * math.pi / 180)), -(radius * math.cos(75 * math.pi / 180))))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU4')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((0.0, -((radius + offset) * math.sin(15 * math.pi / 180)), -((radius + offset) * math.cos(15 * math.pi / 180))))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK4')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((0.0, -((radius + offset) * math.sin(45 * math.pi / 180)), -((radius + offset) * math.cos(45 * math.pi / 180))))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL4')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((0.0, -((radius + offset) * math.sin(75 * math.pi / 180)), -((radius + offset) * math.cos(75 * math.pi / 180))))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT4')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((0.0, -(w / 2 - radius * math.sin(15 * math.pi / 180)), -(h / 2 - radius * math.cos(15 * math.pi / 180))))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB4')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((0.0, -(w / 2 - radius * math.sin(45 * math.pi / 180)), -(h / 2 - radius * math.cos(45 * math.pi / 180))))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC4')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((0.0, -(w / 2 - radius * math.sin(75 * math.pi / 180)), -(h / 2 - radius * math.cos(75 * math.pi / 180))))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR4')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((0.0, -(w / 2 - (radius + offset) * math.sin(15 * math.pi / 180)), -(h / 2 - (radius + offset) * math.cos(15 * math.pi / 180))))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE4')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((0.0, -(w / 2 - (radius + offset) * math.sin(45 * math.pi / 180)), -(h / 2 - (radius + offset) * math.cos(45 * math.pi / 180))))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF4')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((0.0, -(w / 2 - (radius + offset) * math.sin(75 * math.pi / 180)), -(h / 2 - (radius + offset) * math.cos(75 * math.pi / 180))))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS4')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((0.0, -(0.75 / 2 * w), -(1.25 / 2 * s_hex)))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO4')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((0.0, -(w / 8), -(1.75 / 2 * s_hex)))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN4')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((0.0, -0.0, -(radius + offset / 2)))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV4')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((0.0, -((radius + offset / 2) * math.sin(30 * math.pi / 180)), -((radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW4')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((0.0, -((radius + offset / 2) * math.sin(60 * math.pi / 180)), -((radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX4')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((0.0, -(radius + offset / 2), -0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - (radius + offset / 2)), -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2), -(h / 2 - (radius + offset / 2))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180)), -(h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180)), -(h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON4')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, -(w / 2 - yy * math.sin(30 * math.pi / 180)), -(h / 2 - yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM4')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, -(yy * math.sin(30 * math.pi / 180)), -(yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2), -(s_hex / 4)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR4')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2), -LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM4')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -GG, -0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -0.0, -(s_hex - (s_hex - (radius + offset)) * 0.5)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, 0.0, -(1.25 * s_hex)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC4')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(uu * math.cos(30 * math.pi / 180)), -(uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - uu * math.cos(30 * math.pi / 180)), -(h / 2 - uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG4')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -((w / 2 - (radius + offset)) * 0.5), -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG4')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((0.0, -(radius * math.sin(15 * math.pi / 180)), radius * math.cos(15 * math.pi / 180)))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH3')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((0.0, -(radius * math.sin(45 * math.pi / 180)), radius * math.cos(45 * math.pi / 180)))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI3')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((0.0, -(radius * math.sin(75 * math.pi / 180)), radius * math.cos(75 * math.pi / 180)))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU3')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((0.0, -((radius + offset) * math.sin(15 * math.pi / 180)), (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK3')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((0.0, -((radius + offset) * math.sin(45 * math.pi / 180)), (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL3')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((0.0, -((radius + offset) * math.sin(75 * math.pi / 180)), (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT3')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((0.0, -(w / 2 - radius * math.sin(15 * math.pi / 180)), h / 2 - radius * math.cos(15 * math.pi / 180)))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB3')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((0.0, -(w / 2 - radius * math.sin(45 * math.pi / 180)), h / 2 - radius * math.cos(45 * math.pi / 180)))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC3')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((0.0, -(w / 2 - radius * math.sin(75 * math.pi / 180)), h / 2 - radius * math.cos(75 * math.pi / 180)))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR3')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((0.0, -(w / 2 - (radius + offset) * math.sin(15 * math.pi / 180)), h / 2 - (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE3')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((0.0, -(w / 2 - (radius + offset) * math.sin(45 * math.pi / 180)), h / 2 - (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF3')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((0.0, -(w / 2 - (radius + offset) * math.sin(75 * math.pi / 180)), h / 2 - (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS3')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((0.0, -(0.75 / 2 * w), 1.25 / 2 * s_hex))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO3')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((0.0, -(w / 8), 1.75 / 2 * s_hex))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN3')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((0.0, -0.0, radius + offset / 2))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV3')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((0.0, -((radius + offset / 2) * math.sin(30 * math.pi / 180)), (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW3')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((0.0, -((radius + offset / 2) * math.sin(60 * math.pi / 180)), (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX3')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((0.0, -(radius + offset / 2), 0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - (radius + offset / 2)), h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2), h / 2 - (radius + offset / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180)), h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180)), h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON3')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, -(w / 2 - yy * math.sin(30 * math.pi / 180)), h / 2 - yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM3')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((0.0, -(yy * math.sin(30 * math.pi / 180)), yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2), s_hex / 4))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR3')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2), LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM3')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -GG, 0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -0.0, s_hex - (s_hex - (radius + offset)) * 0.5))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, 0.0, 1.25 * s_hex))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC3')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(uu * math.cos(30 * math.pi / 180)), uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -(w / 2 - uu * math.cos(30 * math.pi / 180)), h / 2 - uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG3')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((0.0, -((w / 2 - (radius + offset)) * 0.5), h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG3')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((d, radius * math.sin(15 * math.pi / 180), radius * math.cos(15 * math.pi / 180)))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGHBF')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((d, radius * math.sin(45 * math.pi / 180), radius * math.cos(45 * math.pi / 180)))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHIBF')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((d, radius * math.sin(75 * math.pi / 180), radius * math.cos(75 * math.pi / 180)))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIUBF')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((d, (radius + offset) * math.sin(15 * math.pi / 180), (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJKBF')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((d, (radius + offset) * math.sin(45 * math.pi / 180), (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKLBF')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((d, (radius + offset) * math.sin(75 * math.pi / 180), (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLTBF')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((d, w / 2 - radius * math.sin(15 * math.pi / 180), h / 2 - radius * math.cos(15 * math.pi / 180)))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeABBF')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((d, w / 2 - radius * math.sin(45 * math.pi / 180), h / 2 - radius * math.cos(45 * math.pi / 180)))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBCBF')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((d, w / 2 - radius * math.sin(75 * math.pi / 180), h / 2 - radius * math.cos(75 * math.pi / 180)))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCRBF')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((d, w / 2 - (radius + offset) * math.sin(15 * math.pi / 180), h / 2 - (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDEBF')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((d, w / 2 - (radius + offset) * math.sin(45 * math.pi / 180), h / 2 - (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEFBF')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((d, w / 2 - (radius + offset) * math.sin(75 * math.pi / 180), h / 2 - (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFSBF')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((d, 0.75 / 2 * w, 1.25 / 2 * s_hex))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQOBF')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((d, w / 8, 1.75 / 2 * s_hex))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQNBF')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((d, 0.0, radius + offset / 2))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOVBF')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((d, (radius + offset / 2) * math.sin(30 * math.pi / 180), (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OWBF')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((d, (radius + offset / 2) * math.sin(60 * math.pi / 180), (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OXBF')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d, radius + offset / 2, 0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOYBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - (radius + offset / 2), h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOLBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2, h / 2 - (radius + offset / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOOBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180), h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOMBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180), h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeONBF')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, w / 2 - yy * math.sin(30 * math.pi / 180), h / 2 - yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQMBF')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, yy * math.sin(30 * math.pi / 180), yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQPBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2, s_hex / 4))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeORBF')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2, LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRMBF')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, GG, 0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRVBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, 0.0, s_hex - (s_hex - (radius + offset)) * 0.5))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQBBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, 0.0, 1.25 * s_hex))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQCBF')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, uu * math.cos(30 * math.pi / 180), uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQFBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - uu * math.cos(30 * math.pi / 180), h / 2 - uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQGBF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, (w / 2 - (radius + offset)) * 0.5, h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVGBF')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((d, radius * math.sin(15 * math.pi / 180), -(radius * math.cos(15 * math.pi / 180))))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH2BF')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((d, radius * math.sin(45 * math.pi / 180), -(radius * math.cos(45 * math.pi / 180))))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI2BF')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((d, radius * math.sin(75 * math.pi / 180), -(radius * math.cos(75 * math.pi / 180))))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU2BF')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((d, (radius + offset) * math.sin(15 * math.pi / 180), -((radius + offset) * math.cos(15 * math.pi / 180))))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK2BF')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((d, (radius + offset) * math.sin(45 * math.pi / 180), -((radius + offset) * math.cos(45 * math.pi / 180))))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL2BF')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((d, (radius + offset) * math.sin(75 * math.pi / 180), -((radius + offset) * math.cos(75 * math.pi / 180))))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT2BF')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((d, w / 2 - radius * math.sin(15 * math.pi / 180), -(h / 2 - radius * math.cos(15 * math.pi / 180))))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB2BF')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((d, w / 2 - radius * math.sin(45 * math.pi / 180), -(h / 2 - radius * math.cos(45 * math.pi / 180))))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC2BF')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((d, w / 2 - radius * math.sin(75 * math.pi / 180), -(h / 2 - radius * math.cos(75 * math.pi / 180))))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR2BF')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((d, w / 2 - (radius + offset) * math.sin(15 * math.pi / 180), -(h / 2 - (radius + offset) * math.cos(15 * math.pi / 180))))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE2BF')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((d, w / 2 - (radius + offset) * math.sin(45 * math.pi / 180), -(h / 2 - (radius + offset) * math.cos(45 * math.pi / 180))))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF2BF')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((d, w / 2 - (radius + offset) * math.sin(75 * math.pi / 180), -(h / 2 - (radius + offset) * math.cos(75 * math.pi / 180))))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS2BF')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((d, 0.75 / 2 * w, -(1.25 / 2 * s_hex)))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO2BF')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((d, w / 8, -(1.75 / 2 * s_hex)))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN2BF')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((d, 0.0, -(radius + offset / 2)))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV2BF')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((d, (radius + offset / 2) * math.sin(30 * math.pi / 180), -((radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW2BF')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((d, (radius + offset / 2) * math.sin(60 * math.pi / 180), -((radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX2BF')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d, radius + offset / 2, -0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - (radius + offset / 2), -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2, -(h / 2 - (radius + offset / 2))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180), -(h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180), -(h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON2BF')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, w / 2 - yy * math.sin(30 * math.pi / 180), -(h / 2 - yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM2BF')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, yy * math.sin(30 * math.pi / 180), -(yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2, -(s_hex / 4)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR2BF')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2, -LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM2BF')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, GG, -0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, 0.0, -(s_hex - (s_hex - (radius + offset)) * 0.5)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, 0.0, -(1.25 * s_hex)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC2BF')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, uu * math.cos(30 * math.pi / 180), -(uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, w / 2 - uu * math.cos(30 * math.pi / 180), -(h / 2 - uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG2BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, (w / 2 - (radius + offset)) * 0.5, -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG2BF')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((d, -(radius * math.sin(15 * math.pi / 180)), -(radius * math.cos(15 * math.pi / 180))))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH4BF')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((d, -(radius * math.sin(45 * math.pi / 180)), -(radius * math.cos(45 * math.pi / 180))))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI4BF')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((d, -(radius * math.sin(75 * math.pi / 180)), -(radius * math.cos(75 * math.pi / 180))))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU4BF')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((d, -((radius + offset) * math.sin(15 * math.pi / 180)), -((radius + offset) * math.cos(15 * math.pi / 180))))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK4BF')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((d, -((radius + offset) * math.sin(45 * math.pi / 180)), -((radius + offset) * math.cos(45 * math.pi / 180))))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL4BF')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((d, -((radius + offset) * math.sin(75 * math.pi / 180)), -((radius + offset) * math.cos(75 * math.pi / 180))))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT4BF')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((d, -(w / 2 - radius * math.sin(15 * math.pi / 180)), -(h / 2 - radius * math.cos(15 * math.pi / 180))))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB4BF')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((d, -(w / 2 - radius * math.sin(45 * math.pi / 180)), -(h / 2 - radius * math.cos(45 * math.pi / 180))))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC4BF')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((d, -(w / 2 - radius * math.sin(75 * math.pi / 180)), -(h / 2 - radius * math.cos(75 * math.pi / 180))))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR4BF')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((d, -(w / 2 - (radius + offset) * math.sin(15 * math.pi / 180)), -(h / 2 - (radius + offset) * math.cos(15 * math.pi / 180))))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE4BF')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((d, -(w / 2 - (radius + offset) * math.sin(45 * math.pi / 180)), -(h / 2 - (radius + offset) * math.cos(45 * math.pi / 180))))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF4BF')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((d, -(w / 2 - (radius + offset) * math.sin(75 * math.pi / 180)), -(h / 2 - (radius + offset) * math.cos(75 * math.pi / 180))))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS4BF')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((d, -(0.75 / 2 * w), -(1.25 / 2 * s_hex)))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO4BF')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((d, -(w / 8), -(1.75 / 2 * s_hex)))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN4BF')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((d, -0.0, -(radius + offset / 2)))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV4BF')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((d, -((radius + offset / 2) * math.sin(30 * math.pi / 180)), -((radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW4BF')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((d, -((radius + offset / 2) * math.sin(60 * math.pi / 180)), -((radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX4BF')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d, -(radius + offset / 2), -0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - (radius + offset / 2)), -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2), -(h / 2 - (radius + offset / 2))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180)), -(h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180)), -(h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON4BF')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, -(w / 2 - yy * math.sin(30 * math.pi / 180)), -(h / 2 - yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM4BF')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, -(yy * math.sin(30 * math.pi / 180)), -(yy * math.cos(30 * math.pi / 180))))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2), -(s_hex / 4)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR4BF')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2), -LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM4BF')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -GG, -0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -0.0, -(s_hex - (s_hex - (radius + offset)) * 0.5)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, 0.0, -(1.25 * s_hex)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC4BF')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(uu * math.cos(30 * math.pi / 180)), -(uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - uu * math.cos(30 * math.pi / 180)), -(h / 2 - uu * math.sin(30 * math.pi / 180))))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG4BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -((w / 2 - (radius + offset)) * 0.5), -(h / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG4BF')
    ed2 = HexPackPart.edges
    edgeGH = ed2.findAt((d, -(radius * math.sin(15 * math.pi / 180)), radius * math.cos(15 * math.pi / 180)))
    edgeGIindex = edgeGH.index
    HexPackPart.Set(edges=ed2[edgeGIindex:edgeGIindex + 1], name='EdgeGH3BF')
    ed3 = HexPackPart.edges
    edgeHI = ed3.findAt((d, -(radius * math.sin(45 * math.pi / 180)), radius * math.cos(45 * math.pi / 180)))
    edgeHJindex = edgeHI.index
    HexPackPart.Set(edges=ed3[edgeHJindex:edgeHJindex + 1], name='EdgeHI3BF')
    ed4 = HexPackPart.edges
    edgeIU = ed4.findAt((d, -(radius * math.sin(75 * math.pi / 180)), radius * math.cos(75 * math.pi / 180)))
    edgeIKindex = edgeIU.index
    HexPackPart.Set(edges=ed4[edgeIKindex:edgeIKindex + 1], name='EdgeIU3BF')
    ed5 = HexPackPart.edges
    edgeJK = ed5.findAt((d, -((radius + offset) * math.sin(15 * math.pi / 180)), (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeJLindex = edgeJK.index
    HexPackPart.Set(edges=ed5[edgeJLindex:edgeJLindex + 1], name='EdgeJK3BF')
    ed6 = HexPackPart.edges
    edgeKL = ed6.findAt((d, -((radius + offset) * math.sin(45 * math.pi / 180)), (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeKUindex = edgeKL.index
    HexPackPart.Set(edges=ed6[edgeKUindex:edgeKUindex + 1], name='EdgeKL3BF')
    ed7 = HexPackPart.edges
    edgeLT = ed7.findAt((d, -((radius + offset) * math.sin(75 * math.pi / 180)), (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeIZindex = edgeLT.index
    HexPackPart.Set(edges=ed4[edgeIZindex:edgeIZindex + 1], name='EdgeLT3BF')
    ed8 = HexPackPart.edges
    edgeYZ = ed8.findAt((d, -(w / 2 - radius * math.sin(15 * math.pi / 180)), h / 2 - radius * math.cos(15 * math.pi / 180)))
    edgeXYindex = edgeYZ.index
    HexPackPart.Set(edges=ed8[edgeXYindex:edgeXYindex + 1], name='EdgeAB3BF')
    ed9 = HexPackPart.edges
    edgeBC = ed9.findAt((d, -(w / 2 - radius * math.sin(45 * math.pi / 180)), h / 2 - radius * math.cos(45 * math.pi / 180)))
    edgeBDindex = edgeBC.index
    HexPackPart.Set(edges=ed9[edgeBDindex:edgeBDindex + 1], name='EdgeBC3BF')
    ed10 = HexPackPart.edges
    edgeCR = ed10.findAt((d, -(w / 2 - radius * math.sin(75 * math.pi / 180)), h / 2 - radius * math.cos(75 * math.pi / 180)))
    edgeRCindex = edgeCR.index
    HexPackPart.Set(edges=ed10[edgeRCindex:edgeRCindex + 1], name='EdgeCR3BF')
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((d, -(w / 2 - (radius + offset) * math.sin(15 * math.pi / 180)), h / 2 - (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeDFindex = edgeDE.index
    HexPackPart.Set(edges=ed11[edgeDFindex:edgeDFindex + 1], name='EdgeDE3BF')
    ed12 = HexPackPart.edges
    edgeEF = ed12.findAt((d, -(w / 2 - (radius + offset) * math.sin(45 * math.pi / 180)), h / 2 - (radius + offset) * math.cos(45 * math.pi / 180)))
    edgeESindex = edgeEF.index
    HexPackPart.Set(edges=ed6[edgeESindex:edgeESindex + 1], name='EdgeEF3BF')
    ed13 = HexPackPart.edges
    edgeFS = ed13.findAt((d, -(w / 2 - (radius + offset) * math.sin(75 * math.pi / 180)), h / 2 - (radius + offset) * math.cos(75 * math.pi / 180)))
    edgeFRindex = edgeFS.index
    HexPackPart.Set(edges=ed13[edgeFRindex:edgeFRindex + 1], name='EdgeFS3BF')
    ed14 = HexPackPart.edges
    edgeQO = ed13.findAt((d, -(0.75 / 2 * w), 1.25 / 2 * s_hex))
    edgeRRindex = edgeQO.index
    HexPackPart.Set(edges=ed13[edgeRRindex:edgeRRindex + 1], name='EdgeQO3BF')
    ed15 = HexPackPart.edges
    edgeQN = ed15.findAt((d, -(w / 8), 1.75 / 2 * s_hex))
    edgeNOindex = edgeQN.index
    HexPackPart.Set(edges=ed13[edgeNOindex:edgeNOindex + 1], name='EdgeQN3BF')
    ed17 = HexPackPart.edges
    edgeOV = ed17.findAt((d, -0.0, radius + offset / 2))
    edgeOWindex = edgeOV.index
    HexPackPart.Set(edges=ed17[edgeOWindex:edgeOWindex + 1], name='EdgeOV3BF')
    ed18 = HexPackPart.edges
    edgeOW = ed18.findAt((d, -((radius + offset / 2) * math.sin(30 * math.pi / 180)), (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeOVindex = edgeOW.index
    HexPackPart.Set(edges=ed18[edgeOVindex:edgeOVindex + 1], name='Egde OW3BF')
    ed19 = HexPackPart.edges
    edgeOX = ed19.findAt((d, -((radius + offset / 2) * math.sin(60 * math.pi / 180)), (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeXVindex = edgeOX.index
    HexPackPart.Set(edges=ed18[edgeXVindex:edgeXVindex + 1], name='Egde OX3BF')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d, -(radius + offset / 2), 0.0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='EdgeOY3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - (radius + offset / 2)), h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOL3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2), h / 2 - (radius + offset / 2)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed17[edgeLOindex:edgeLOindex + 1], name='EdgeOO3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - (radius + offset / 2) * math.sin(30 * math.pi / 180)), h / 2 - (radius + offset / 2) * math.cos(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOM3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - (radius + offset / 2) * math.sin(60 * math.pi / 180)), h / 2 - (radius + offset / 2) * math.cos(60 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeON3BF')
    yy = 0.5 * math.sqrt(3) * s_hex - (0.5 * math.sqrt(3) * s_hex - (radius + offset)) * 0.5
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, -(w / 2 - yy * math.sin(30 * math.pi / 180)), h / 2 - yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQM3BF')
    ed16 = HexPackPart.edges
    edgeQM = ed16.findAt((d, -(yy * math.sin(30 * math.pi / 180)), yy * math.cos(30 * math.pi / 180)))
    edgeQPindex = edgeQM.index
    HexPackPart.Set(edges=ed16[edgeQPindex:edgeQPindex + 1], name='EdgeQP3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2), s_hex / 4))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeOR3BF')
    LL = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2), LL))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRM3BF')
    GG = w / 2 - (w / 2 - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -GG, 0.0))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeRV3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -0.0, s_hex - (s_hex - (radius + offset)) * 0.5))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQB3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, 0.0, 1.25 * s_hex))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQC3BF')
    uu = s_hex - (s_hex - (radius + offset)) * 0.5
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(uu * math.cos(30 * math.pi / 180)), uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQF3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -(w / 2 - uu * math.cos(30 * math.pi / 180)), h / 2 - uu * math.sin(30 * math.pi / 180)))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeQG3BF')
    ed21 = HexPackPart.edges
    edgeOL = ed21.findAt((d, -((w / 2 - (radius + offset)) * 0.5), h / 2))
    edgeLOindex = edgeOL.index
    HexPackPart.Set(edges=ed21[edgeLOindex:edgeLOindex + 1], name='EdgeVG3BF')
    dv = s_hex - (radius + offset)
    ds = 0.5 * math.sqrt(3) * s_hex - (radius + offset)
    l1 = ds / n_rad
    ln = dv / n_rad
    HexPackPart.SetByBoolean(name='cohesive_zone_radial_edges', sets=(HexPackPart.sets['EdgeOL'],
     HexPackPart.sets['EdgeOL2'], HexPackPart.sets['EdgeOL2BF'], HexPackPart.sets['EdgeOL3'],
     HexPackPart.sets['EdgeOL3BF'], HexPackPart.sets['EdgeOL4'], HexPackPart.sets['EdgeOL4BF'],
     HexPackPart.sets['EdgeOLBF'], HexPackPart.sets['EdgeOM'], HexPackPart.sets['EdgeOM2'],
     HexPackPart.sets['EdgeOM2BF'], HexPackPart.sets['EdgeOM3'], HexPackPart.sets['EdgeOM3BF'],
     HexPackPart.sets['EdgeOM4'], HexPackPart.sets['EdgeOM4BF'], HexPackPart.sets['EdgeOMBF'],
     HexPackPart.sets['EdgeON'], HexPackPart.sets['EdgeON2'], HexPackPart.sets['EdgeON2BF'],
     HexPackPart.sets['EdgeON3'], HexPackPart.sets['EdgeON3BF'], HexPackPart.sets['EdgeON4'],
     HexPackPart.sets['EdgeON4BF'], HexPackPart.sets['EdgeONBF'], HexPackPart.sets['EdgeOO'],
     HexPackPart.sets['EdgeOO2'], HexPackPart.sets['EdgeOO2BF'], HexPackPart.sets['EdgeOO3'],
     HexPackPart.sets['EdgeOO3BF'], HexPackPart.sets['EdgeOO4'], HexPackPart.sets['EdgeOO4BF'],
     HexPackPart.sets['EdgeOOBF'], HexPackPart.sets['EdgeOV2'], HexPackPart.sets['EdgeOV2BF'],
     HexPackPart.sets['EdgeOV3'], HexPackPart.sets['EdgeOV3BF'], HexPackPart.sets['EdgeOV4'],
     HexPackPart.sets['EdgeOV4BF'], HexPackPart.sets['EdgeOVBF'], HexPackPart.sets['EdgeOY'],
     HexPackPart.sets['EdgeOY2'], HexPackPart.sets['EdgeOY2BF'], HexPackPart.sets['EdgeOY3'],
     HexPackPart.sets['EdgeOY3BF'], HexPackPart.sets['EdgeOY4'], HexPackPart.sets['EdgeOY4BF'],
     HexPackPart.sets['EdgeOYBF'], HexPackPart.sets['Egde OXBF'], HexPackPart.sets['Egde OX4BF'],
     HexPackPart.sets['Egde OX4'], HexPackPart.sets['Egde OX3BF'], HexPackPart.sets['Egde OX3'],
     HexPackPart.sets['Egde OX2BF'], HexPackPart.sets['Egde OX2'], HexPackPart.sets['Egde OX'],
     HexPackPart.sets['Egde OWBF'], HexPackPart.sets['Egde OW4BF'], HexPackPart.sets['Egde OW4'],
     HexPackPart.sets['Egde OW3BF'], HexPackPart.sets['Egde OW3'], HexPackPart.sets['Egde OW2BF'],
     HexPackPart.sets['Egde OW2'], HexPackPart.sets['Egde OW']))
    r = HexPackPart.sets['cohesive_zone_radial_edges'].edges
    HexPackPart.seedEdgeByNumber(edges=r, number=1, constraint=FINER)
    HexPackPart.SetByBoolean(name='seed_by_number_edges', sets=(HexPackPart.sets['EdgeQBBF'],
     HexPackPart.sets['EdgeQB'], HexPackPart.sets['EdgeQB2'], HexPackPart.sets['EdgeQB2BF'],
     HexPackPart.sets['EdgeQB3'], HexPackPart.sets['EdgeQB3BF'], HexPackPart.sets['EdgeQB4'],
     HexPackPart.sets['EdgeQB4BF'], HexPackPart.sets['EdgeQF'], HexPackPart.sets['EdgeQF2'],
     HexPackPart.sets['EdgeQF2BF'], HexPackPart.sets['EdgeQF3'], HexPackPart.sets['EdgeQF3BF'],
     HexPackPart.sets['EdgeQF4'], HexPackPart.sets['EdgeQF4BF'], HexPackPart.sets['EdgeQFBF'],
     HexPackPart.sets['EdgeQG'], HexPackPart.sets['EdgeQGBF'], HexPackPart.sets['EdgeQG4BF'],
     HexPackPart.sets['EdgeQG4'], HexPackPart.sets['EdgeQG3BF'], HexPackPart.sets['EdgeQG3'],
     HexPackPart.sets['EdgeQG2BF'], HexPackPart.sets['EdgeQG2'], HexPackPart.sets['EdgeQM'],
     HexPackPart.sets['EdgeQMBF'], HexPackPart.sets['EdgeQM4BF'], HexPackPart.sets['EdgeQM4'],
     HexPackPart.sets['EdgeQM3BF'], HexPackPart.sets['EdgeQM3'], HexPackPart.sets['EdgeQM2BF'],
     HexPackPart.sets['EdgeQM2'], HexPackPart.sets['EdgeQP'], HexPackPart.sets['EdgeQPBF'],
     HexPackPart.sets['EdgeQP4BF'], HexPackPart.sets['EdgeQP4'], HexPackPart.sets['EdgeQP3BF'],
     HexPackPart.sets['EdgeQP3'], HexPackPart.sets['EdgeQP2BF'], HexPackPart.sets['EdgeQP2'],
     HexPackPart.sets['EdgeRM'], HexPackPart.sets['EdgeRM2'], HexPackPart.sets['EdgeRM2BF'],
     HexPackPart.sets['EdgeRM3'], HexPackPart.sets['EdgeRM3BF'], HexPackPart.sets['EdgeRM4'],
     HexPackPart.sets['EdgeRM4BF'], HexPackPart.sets['EdgeRMBF'], HexPackPart.sets['EdgeRV'],
     HexPackPart.sets['EdgeRV2'], HexPackPart.sets['EdgeRV2BF'], HexPackPart.sets['EdgeRV3'],
     HexPackPart.sets['EdgeRV3BF'], HexPackPart.sets['EdgeRV4'], HexPackPart.sets['EdgeRV4BF'],
     HexPackPart.sets['EdgeRVBF'], HexPackPart.sets['EdgeVG'], HexPackPart.sets['EdgeVG2'],
     HexPackPart.sets['EdgeVG2BF'], HexPackPart.sets['EdgeVG3'], HexPackPart.sets['EdgeVG3BF'],
     HexPackPart.sets['EdgeVG4'], HexPackPart.sets['EdgeVG4BF'], HexPackPart.sets['EdgeVGBF']))
    r = HexPackPart.sets['seed_by_number_edges'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end1Edges=r, minSize=l1, maxSize=ln, constraint=FINER)
    HexPackPart.SetByBoolean(name='all_matrix_coh_zone_edges', sets=(HexPackPart.sets['EdgeAB'],
     HexPackPart.sets['EdgeAB2'], HexPackPart.sets['EdgeAB2BF'], HexPackPart.sets['EdgeAB3'],
     HexPackPart.sets['EdgeAB3BF'], HexPackPart.sets['EdgeAB4'], HexPackPart.sets['EdgeAB4BF'],
     HexPackPart.sets['EdgeABBF'], HexPackPart.sets['EdgeBC'], HexPackPart.sets['EdgeBC2'],
     HexPackPart.sets['EdgeBC2BF'], HexPackPart.sets['EdgeBC3'], HexPackPart.sets['EdgeBC3BF'],
     HexPackPart.sets['EdgeBC4'], HexPackPart.sets['EdgeBC4BF'], HexPackPart.sets['EdgeBCBF'],
     HexPackPart.sets['EdgeCR'], HexPackPart.sets['EdgeCR2'], HexPackPart.sets['EdgeCR2BF'],
     HexPackPart.sets['EdgeCR3'], HexPackPart.sets['EdgeCR3BF'], HexPackPart.sets['EdgeCR4'],
     HexPackPart.sets['EdgeCR4BF'], HexPackPart.sets['EdgeCRBF'], HexPackPart.sets['EdgeDE'],
     HexPackPart.sets['EdgeDE2'], HexPackPart.sets['EdgeDE2BF'], HexPackPart.sets['EdgeDE3'],
     HexPackPart.sets['EdgeDE3BF'], HexPackPart.sets['EdgeDE4'], HexPackPart.sets['EdgeDE4BF'],
     HexPackPart.sets['EdgeDEBF'], HexPackPart.sets['EdgeEF'], HexPackPart.sets['EdgeEF2'],
     HexPackPart.sets['EdgeEF2BF'], HexPackPart.sets['EdgeEF3'], HexPackPart.sets['EdgeEF3BF'],
     HexPackPart.sets['EdgeEF4'], HexPackPart.sets['EdgeEF4BF'], HexPackPart.sets['EdgeEFBF'],
     HexPackPart.sets['EdgeFS'], HexPackPart.sets['EdgeFS2'], HexPackPart.sets['EdgeFS2BF'],
     HexPackPart.sets['EdgeFS3'], HexPackPart.sets['EdgeFS3BF'], HexPackPart.sets['EdgeFS4'],
     HexPackPart.sets['EdgeFS4BF'], HexPackPart.sets['EdgeFSBF'], HexPackPart.sets['EdgeGH'],
     HexPackPart.sets['EdgeGH2'], HexPackPart.sets['EdgeGH2BF'], HexPackPart.sets['EdgeGH3'],
     HexPackPart.sets['EdgeGH3BF'], HexPackPart.sets['EdgeGH4'], HexPackPart.sets['EdgeGH4BF'],
     HexPackPart.sets['EdgeGHBF'], HexPackPart.sets['EdgeHI'], HexPackPart.sets['EdgeHI2'],
     HexPackPart.sets['EdgeHI2BF'], HexPackPart.sets['EdgeHI3'], HexPackPart.sets['EdgeHI3BF'],
     HexPackPart.sets['EdgeHI4'], HexPackPart.sets['EdgeHI4BF'], HexPackPart.sets['EdgeHIBF'],
     HexPackPart.sets['EdgeIU'], HexPackPart.sets['EdgeIU2'], HexPackPart.sets['EdgeIU2BF'],
     HexPackPart.sets['EdgeIU3'], HexPackPart.sets['EdgeIU3BF'], HexPackPart.sets['EdgeIU4'],
     HexPackPart.sets['EdgeIU4BF'], HexPackPart.sets['EdgeIUBF'], HexPackPart.sets['EdgeJK'],
     HexPackPart.sets['EdgeJK2'], HexPackPart.sets['EdgeJK2BF'], HexPackPart.sets['EdgeJK3'],
     HexPackPart.sets['EdgeJK3BF'], HexPackPart.sets['EdgeJK4'], HexPackPart.sets['EdgeJK4BF'],
     HexPackPart.sets['EdgeJKBF'], HexPackPart.sets['EdgeKL'], HexPackPart.sets['EdgeKL2'],
     HexPackPart.sets['EdgeKL2BF'], HexPackPart.sets['EdgeKL3'], HexPackPart.sets['EdgeKL3BF'],
     HexPackPart.sets['EdgeKL4'], HexPackPart.sets['EdgeKL4BF'], HexPackPart.sets['EdgeKLBF'],
     HexPackPart.sets['EdgeLT'], HexPackPart.sets['EdgeLT2'], HexPackPart.sets['EdgeLT2BF'],
     HexPackPart.sets['EdgeLT3'], HexPackPart.sets['EdgeLT3BF'], HexPackPart.sets['EdgeLT4'],
     HexPackPart.sets['EdgeLT4BF'], HexPackPart.sets['EdgeLTBF'], HexPackPart.sets['EdgeOL'],
     HexPackPart.sets['EdgeOL2'], HexPackPart.sets['EdgeOL2BF'], HexPackPart.sets['EdgeOL3'],
     HexPackPart.sets['EdgeOL3BF'], HexPackPart.sets['EdgeOL4'], HexPackPart.sets['EdgeOL4BF'],
     HexPackPart.sets['EdgeOLBF'], HexPackPart.sets['EdgeOM'], HexPackPart.sets['EdgeOM2'],
     HexPackPart.sets['EdgeOM2BF'], HexPackPart.sets['EdgeOM3'], HexPackPart.sets['EdgeOM3BF'],
     HexPackPart.sets['EdgeOM4'], HexPackPart.sets['EdgeOM4BF'], HexPackPart.sets['EdgeOMBF'],
     HexPackPart.sets['EdgeON'], HexPackPart.sets['EdgeON2'], HexPackPart.sets['EdgeON2BF'],
     HexPackPart.sets['EdgeON3'], HexPackPart.sets['EdgeON3BF'], HexPackPart.sets['EdgeON4'],
     HexPackPart.sets['EdgeON4BF'], HexPackPart.sets['EdgeONBF'], HexPackPart.sets['EdgeOO'],
     HexPackPart.sets['EdgeOO2'], HexPackPart.sets['EdgeOO2BF'], HexPackPart.sets['EdgeOO3'],
     HexPackPart.sets['EdgeOO3BF'], HexPackPart.sets['EdgeOO4'], HexPackPart.sets['EdgeOO4BF'],
     HexPackPart.sets['EdgeOOBF'], HexPackPart.sets['EdgeOR'], HexPackPart.sets['EdgeOR2'],
     HexPackPart.sets['EdgeOR2BF'], HexPackPart.sets['EdgeOR3'], HexPackPart.sets['EdgeOR3BF'],
     HexPackPart.sets['EdgeOR4'], HexPackPart.sets['EdgeOR4BF'], HexPackPart.sets['EdgeORBF'],
     HexPackPart.sets['EdgeOV'], HexPackPart.sets['EdgeOV2'], HexPackPart.sets['EdgeOV2BF'],
     HexPackPart.sets['EdgeOV3'], HexPackPart.sets['EdgeOV3BF'], HexPackPart.sets['EdgeOV4'],
     HexPackPart.sets['EdgeOV4BF'], HexPackPart.sets['EdgeOVBF'], HexPackPart.sets['EdgeOY'],
     HexPackPart.sets['EdgeOY2'], HexPackPart.sets['EdgeOY2BF'], HexPackPart.sets['EdgeOY3'],
     HexPackPart.sets['EdgeOY3BF'], HexPackPart.sets['EdgeOY4'], HexPackPart.sets['EdgeOY4BF'],
     HexPackPart.sets['EdgeOYBF'], HexPackPart.sets['EdgeQB'], HexPackPart.sets['EdgeQB2'],
     HexPackPart.sets['EdgeQB2BF'], HexPackPart.sets['EdgeQB3'], HexPackPart.sets['EdgeQB3BF'],
     HexPackPart.sets['EdgeQB4'], HexPackPart.sets['EdgeQB4BF'], HexPackPart.sets['EdgeQBBF'],
     HexPackPart.sets['EdgeQC'], HexPackPart.sets['EdgeQC2'], HexPackPart.sets['EdgeQC2BF'],
     HexPackPart.sets['EdgeQC3'], HexPackPart.sets['EdgeQC3BF'], HexPackPart.sets['EdgeQC4'],
     HexPackPart.sets['EdgeQC4BF'], HexPackPart.sets['EdgeQCBF'], HexPackPart.sets['EdgeQF'],
     HexPackPart.sets['EdgeQF2'], HexPackPart.sets['EdgeQF2BF'], HexPackPart.sets['EdgeQF3'],
     HexPackPart.sets['EdgeQF3BF'], HexPackPart.sets['EdgeQF4'], HexPackPart.sets['EdgeQF4BF'],
     HexPackPart.sets['EdgeQFBF'], HexPackPart.sets['EdgeQG'], HexPackPart.sets['EdgeQG2'],
     HexPackPart.sets['EdgeQG2BF'], HexPackPart.sets['EdgeQG3'], HexPackPart.sets['EdgeQG3BF'],
     HexPackPart.sets['EdgeQG4'], HexPackPart.sets['EdgeQG4BF'], HexPackPart.sets['EdgeQGBF'],
     HexPackPart.sets['EdgeQM'], HexPackPart.sets['EdgeQM2'], HexPackPart.sets['EdgeQM2BF'],
     HexPackPart.sets['EdgeQM3'], HexPackPart.sets['EdgeQM3BF'], HexPackPart.sets['EdgeQM4'],
     HexPackPart.sets['EdgeQM4BF'], HexPackPart.sets['EdgeQMBF'], HexPackPart.sets['EdgeQN'],
     HexPackPart.sets['EdgeQN2'], HexPackPart.sets['EdgeQN2BF'], HexPackPart.sets['EdgeQN3'],
     HexPackPart.sets['EdgeQN3BF'], HexPackPart.sets['EdgeQN4'], HexPackPart.sets['EdgeQN4BF'],
     HexPackPart.sets['EdgeQNBF'], HexPackPart.sets['EdgeQO'], HexPackPart.sets['EdgeQO2'],
     HexPackPart.sets['EdgeQO2BF'], HexPackPart.sets['EdgeQO3'], HexPackPart.sets['EdgeQO3BF'],
     HexPackPart.sets['EdgeQO4'], HexPackPart.sets['EdgeQO4BF'], HexPackPart.sets['EdgeQOBF'],
     HexPackPart.sets['EdgeQP'], HexPackPart.sets['EdgeQP2'], HexPackPart.sets['EdgeQP2BF'],
     HexPackPart.sets['EdgeQP3'], HexPackPart.sets['EdgeQP3BF'], HexPackPart.sets['EdgeQP4'],
     HexPackPart.sets['EdgeQP4BF'], HexPackPart.sets['EdgeQPBF'], HexPackPart.sets['EdgeRM'],
     HexPackPart.sets['EdgeRM2'], HexPackPart.sets['EdgeRM2BF'], HexPackPart.sets['EdgeRM3'],
     HexPackPart.sets['EdgeRM3BF'], HexPackPart.sets['EdgeRM4'], HexPackPart.sets['EdgeRM4BF'],
     HexPackPart.sets['EdgeRMBF'], HexPackPart.sets['EdgeRV'], HexPackPart.sets['EdgeRV2'],
     HexPackPart.sets['EdgeRV2BF'], HexPackPart.sets['EdgeRV3'], HexPackPart.sets['EdgeRV3BF'],
     HexPackPart.sets['EdgeRV4'], HexPackPart.sets['EdgeRV4BF'], HexPackPart.sets['EdgeRVBF'],
     HexPackPart.sets['EdgeVG'], HexPackPart.sets['EdgeVG2'], HexPackPart.sets['EdgeVG2BF'],
     HexPackPart.sets['EdgeVG3'], HexPackPart.sets['EdgeVG3BF'], HexPackPart.sets['EdgeVG4'],
     HexPackPart.sets['EdgeVG4BF'], HexPackPart.sets['EdgeVGBF'], HexPackPart.sets['Egde OW'],
     HexPackPart.sets['Egde OW2'], HexPackPart.sets['Egde OW2BF'], HexPackPart.sets['Egde OW3'],
     HexPackPart.sets['Egde OW3BF'], HexPackPart.sets['Egde OW4'], HexPackPart.sets['Egde OW4BF'],
     HexPackPart.sets['Egde OWBF'], HexPackPart.sets['Egde OX'], HexPackPart.sets['Egde OX2'],
     HexPackPart.sets['Egde OX2BF'], HexPackPart.sets['Egde OX3'], HexPackPart.sets['Egde OX3BF'],
     HexPackPart.sets['Egde OX4'], HexPackPart.sets['Egde OX4BF'], HexPackPart.sets['Egde OXBF']))
    HexPackPart.SetByBoolean(name='seed_by_bias', operation=DIFFERENCE, sets=(
     HexPackPart.sets['all_matrix_coh_zone_edges'], HexPackPart.sets['cohesive_zone_radial_edges'],
     HexPackPart.sets['seed_by_number_edges']))
    HexPackPart.SetByBoolean(name='seed_by_min_max_size', sets=(HexPackPart.sets['EdgeDE'],
     HexPackPart.sets['EdgeDE2'], HexPackPart.sets['EdgeDE2BF'], HexPackPart.sets['EdgeDE3'],
     HexPackPart.sets['EdgeDE3BF'], HexPackPart.sets['EdgeDE4'], HexPackPart.sets['EdgeDE4BF'],
     HexPackPart.sets['EdgeDEBF'], HexPackPart.sets['EdgeEF'], HexPackPart.sets['EdgeEF2'],
     HexPackPart.sets['EdgeEF2BF'], HexPackPart.sets['EdgeEF3'], HexPackPart.sets['EdgeEF3BF'],
     HexPackPart.sets['EdgeEF4'], HexPackPart.sets['EdgeEF4BF'], HexPackPart.sets['EdgeEFBF'],
     HexPackPart.sets['EdgeFS'], HexPackPart.sets['EdgeFS2'], HexPackPart.sets['EdgeFS2BF'],
     HexPackPart.sets['EdgeFS3'], HexPackPart.sets['EdgeFS3BF'], HexPackPart.sets['EdgeFS4'],
     HexPackPart.sets['EdgeFS4BF'], HexPackPart.sets['EdgeFSBF'], HexPackPart.sets['EdgeJK'],
     HexPackPart.sets['EdgeJK2'], HexPackPart.sets['EdgeJK2BF'], HexPackPart.sets['EdgeJK3'],
     HexPackPart.sets['EdgeJK3BF'], HexPackPart.sets['EdgeJK4'], HexPackPart.sets['EdgeJK4BF'],
     HexPackPart.sets['EdgeJKBF'], HexPackPart.sets['EdgeKL'], HexPackPart.sets['EdgeKL2'],
     HexPackPart.sets['EdgeKL2BF'], HexPackPart.sets['EdgeKL3'], HexPackPart.sets['EdgeKL3BF'],
     HexPackPart.sets['EdgeKL4'], HexPackPart.sets['EdgeKL4BF'], HexPackPart.sets['EdgeKLBF'],
     HexPackPart.sets['EdgeLT'], HexPackPart.sets['EdgeLT2'], HexPackPart.sets['EdgeLT2BF'],
     HexPackPart.sets['EdgeLT3'], HexPackPart.sets['EdgeLT3BF'], HexPackPart.sets['EdgeLT4'],
     HexPackPart.sets['EdgeLTBF'], HexPackPart.sets['EdgeLT4BF']))
    HexPackPart.SetByBoolean(name='seed_by_bias_number', operation=DIFFERENCE, sets=(
     HexPackPart.sets['seed_by_bias'], HexPackPart.sets['seed_by_min_max_size']))
    HexPackPart.SetByBoolean(name='backface_flipSet', sets=(HexPackPart.sets['EdgeAB3BF'],
     HexPackPart.sets['EdgeAB2BF'], HexPackPart.sets['EdgeAB4BF'], HexPackPart.sets['EdgeABBF'],
     HexPackPart.sets['EdgeCR2BF'], HexPackPart.sets['EdgeCR3BF'], HexPackPart.sets['EdgeCR4BF'],
     HexPackPart.sets['EdgeCRBF'], HexPackPart.sets['EdgeEF2BF'], HexPackPart.sets['EdgeEF3BF'],
     HexPackPart.sets['EdgeEF4BF'], HexPackPart.sets['EdgeEFBF'], HexPackPart.sets['EdgeHI2BF'],
     HexPackPart.sets['EdgeHI3BF'], HexPackPart.sets['EdgeHI4BF'], HexPackPart.sets['EdgeHIBF'],
     HexPackPart.sets['EdgeJK2BF'], HexPackPart.sets['EdgeJK3BF'], HexPackPart.sets['EdgeJK4BF'],
     HexPackPart.sets['EdgeJKBF'], HexPackPart.sets['EdgeLT2BF'], HexPackPart.sets['EdgeLT3BF'],
     HexPackPart.sets['EdgeLTBF'], HexPackPart.sets['EdgeOR2BF'], HexPackPart.sets['EdgeOR3BF'],
     HexPackPart.sets['EdgeOR4BF'], HexPackPart.sets['EdgeORBF'], HexPackPart.sets['EdgeQN2BF'],
     HexPackPart.sets['EdgeQN3BF'], HexPackPart.sets['EdgeQN4BF'], HexPackPart.sets['EdgeQNBF'],
     HexPackPart.sets['EdgeLT4BF']))
    HexPackPart.SetByBoolean(name='frontface_flipSet', sets=(HexPackPart.sets['EdgeAB'],
     HexPackPart.sets['EdgeAB2'], HexPackPart.sets['EdgeAB3'], HexPackPart.sets['EdgeAB4'], HexPackPart.sets['EdgeCR'],
     HexPackPart.sets['EdgeCR2'], HexPackPart.sets['EdgeCR3'], HexPackPart.sets['EdgeCR4'], HexPackPart.sets['EdgeEF'],
     HexPackPart.sets['EdgeEF2'], HexPackPart.sets['EdgeEF3'], HexPackPart.sets['EdgeEF4'], HexPackPart.sets['EdgeHI'],
     HexPackPart.sets['EdgeHI2'], HexPackPart.sets['EdgeHI3'], HexPackPart.sets['EdgeHI4'], HexPackPart.sets['EdgeJK'],
     HexPackPart.sets['EdgeJK2'], HexPackPart.sets['EdgeJK3'], HexPackPart.sets['EdgeJK4'], HexPackPart.sets['EdgeLT'],
     HexPackPart.sets['EdgeLT2'], HexPackPart.sets['EdgeLT3'], HexPackPart.sets['EdgeLT4'], HexPackPart.sets['EdgeOR'],
     HexPackPart.sets['EdgeOR2'], HexPackPart.sets['EdgeOR3'], HexPackPart.sets['EdgeOR4'], HexPackPart.sets['EdgeQN'],
     HexPackPart.sets['EdgeQN2'], HexPackPart.sets['EdgeQN3'], HexPackPart.sets['EdgeQN4']))
    HexPackPart.SetByBoolean(name='front_face_all', sets=(HexPackPart.sets['EdgeAB'],
     HexPackPart.sets['EdgeAB2'], HexPackPart.sets['EdgeAB3'], HexPackPart.sets['EdgeAB4'], HexPackPart.sets['EdgeCR'],
     HexPackPart.sets['EdgeCR2'], HexPackPart.sets['EdgeCR3'], HexPackPart.sets['EdgeCR4'], HexPackPart.sets['EdgeEF'],
     HexPackPart.sets['EdgeEF2'], HexPackPart.sets['EdgeEF3'], HexPackPart.sets['EdgeEF4'], HexPackPart.sets['EdgeHI'],
     HexPackPart.sets['EdgeHI2'], HexPackPart.sets['EdgeHI4'], HexPackPart.sets['EdgeHI3'], HexPackPart.sets['EdgeBC'],
     HexPackPart.sets['EdgeBC2'], HexPackPart.sets['EdgeBC3'], HexPackPart.sets['EdgeBC4'], HexPackPart.sets['EdgeFS'],
     HexPackPart.sets['EdgeFS2'], HexPackPart.sets['EdgeFS3'], HexPackPart.sets['EdgeFS4'], HexPackPart.sets['EdgeGH'],
     HexPackPart.sets['EdgeGH2'], HexPackPart.sets['EdgeGH3'], HexPackPart.sets['EdgeGH4'], HexPackPart.sets['EdgeIU'],
     HexPackPart.sets['EdgeIU2'], HexPackPart.sets['EdgeIU3'], HexPackPart.sets['EdgeIU4'], HexPackPart.sets['EdgeJK'],
     HexPackPart.sets['EdgeJK2'], HexPackPart.sets['EdgeJK3'], HexPackPart.sets['EdgeJK4'], HexPackPart.sets['EdgeKL'],
     HexPackPart.sets['EdgeKL2'], HexPackPart.sets['EdgeKL3'], HexPackPart.sets['EdgeKL4'], HexPackPart.sets['EdgeLT'],
     HexPackPart.sets['EdgeLT2'], HexPackPart.sets['EdgeLT3'], HexPackPart.sets['EdgeQN2'], HexPackPart.sets['EdgeQN3'],
     HexPackPart.sets['EdgeQN4'], HexPackPart.sets['EdgeQO'], HexPackPart.sets['EdgeQO2'], HexPackPart.sets['EdgeQO3'],
     HexPackPart.sets['EdgeQO4'], HexPackPart.sets['EdgeLT4'], HexPackPart.sets['EdgeDE'], HexPackPart.sets['EdgeDE2'],
     HexPackPart.sets['EdgeDE3'], HexPackPart.sets['EdgeDE4'], HexPackPart.sets['EdgeQN'], HexPackPart.sets['EdgeOR'],
     HexPackPart.sets['EdgeOR2'], HexPackPart.sets['EdgeOR3'], HexPackPart.sets['EdgeOR4']))
    HexPackPart.SetByBoolean(name='Set-303', operation=DIFFERENCE, sets=(
     HexPackPart.sets['front_face_all'], HexPackPart.sets['frontface_flipSet']))
    HexPackPart = mdb.models[RVE_modelName].parts['HexPackPart']
    HexPackPart.SetByBoolean(name='front_face_flipright', operation=DIFFERENCE, sets=(
     HexPackPart.sets['front_face_all'], HexPackPart.sets['frontface_flipSet']))
    HexPackPart.SetByBoolean(name='frontface_rightflip', operation=DIFFERENCE, sets=(
     HexPackPart.sets['front_face_all'], HexPackPart.sets['frontface_flipSet']))
    HexPackPart.SetByBoolean(name='frontface_rightflip', operation=DIFFERENCE, sets=(
     HexPackPart.sets['front_face_all'], HexPackPart.sets['frontface_flipSet']))
    HexPackPart.SetByBoolean(name='hex_edges_frontflip', sets=(HexPackPart.sets['EdgeOR'],
     HexPackPart.sets['EdgeOR2'], HexPackPart.sets['EdgeOR3'], HexPackPart.sets['EdgeOR4']))
    HexPackPart.SetByBoolean(name='final_frontFlip', sets=(HexPackPart.sets['frontface_rightflip'],
     HexPackPart.sets['hex_edges_frontflip']))
    HexPackPart.SetByBoolean(name='seed_by_min_max_flip', sets=(HexPackPart.sets['EdgeKL4'],
     HexPackPart.sets['EdgeKL3'], HexPackPart.sets['EdgeKL2'], HexPackPart.sets['EdgeKL'], HexPackPart.sets['EdgeFS4'],
     HexPackPart.sets['EdgeFS3'], HexPackPart.sets['EdgeFS2'], HexPackPart.sets['EdgeFS'], HexPackPart.sets['EdgeDE4'],
     HexPackPart.sets['EdgeDE3'], HexPackPart.sets['EdgeDE2'], HexPackPart.sets['EdgeDE'], HexPackPart.sets['EdgeLTBF'],
     HexPackPart.sets['EdgeLT3BF'], HexPackPart.sets['EdgeLT2BF'], HexPackPart.sets['EdgeLT4BF'],
     HexPackPart.sets['EdgeJKBF'], HexPackPart.sets['EdgeJK4BF'], HexPackPart.sets['EdgeJK3BF'],
     HexPackPart.sets['EdgeJK2BF'], HexPackPart.sets['EdgeEFBF'], HexPackPart.sets['EdgeEF4BF'],
     HexPackPart.sets['EdgeEF3BF'], HexPackPart.sets['EdgeEF2BF']))
    HexPackPart.seedPart(size=radius / 10, deviationFactor=0.1, minSizeFactor=0.1)
    r = HexPackPart.sets['seed_by_min_max_size'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end1Edges=r, minSize=l1, maxSize=ln, constraint=FINER)
    ed11 = HexPackPart.edges
    edgeDE = ed11.findAt((0.0, w / 2 - (radius + offset) * math.sin(15 * math.pi / 180), h / 2 - (radius + offset) * math.cos(15 * math.pi / 180)))
    edgeDFindex = edgeDE.index
    r = HexPackPart.sets['seed_by_min_max_flip'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end2Edges=r, minSize=l1, maxSize=ln, constraint=FINER)
    bias_ratio = HexPackPart.getEdgeSeeds(edgeDE, BIAS_RATIO)
    no_of_seed = HexPackPart.getEdgeSeeds(edgeDE, NUMBER)
    r = HexPackPart.sets['seed_by_bias_number'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end2Edges=r, ratio=bias_ratio, number=no_of_seed, constraint=FINER)
    HexPackPart.SetByBoolean(name='frontface_flipset11', sets=(HexPackPart.sets['EdgeBC'],
     HexPackPart.sets['EdgeBC2'], HexPackPart.sets['EdgeBC3'], HexPackPart.sets['EdgeBC4'], HexPackPart.sets['EdgeGH'],
     HexPackPart.sets['EdgeGH2'], HexPackPart.sets['EdgeGH3'], HexPackPart.sets['EdgeGH4'], HexPackPart.sets['EdgeIU'],
     HexPackPart.sets['EdgeIU2'], HexPackPart.sets['EdgeIU3'], HexPackPart.sets['EdgeIU4']))
    r = HexPackPart.sets['frontface_flipset11'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end1Edges=r, ratio=bias_ratio, number=no_of_seed, constraint=FINER)
    HexPackPart.SetByBoolean(name='backside_flip11', sets=(HexPackPart.sets['EdgeAB2BF'],
     HexPackPart.sets['EdgeAB3BF'], HexPackPart.sets['EdgeAB4BF'], HexPackPart.sets['EdgeABBF'],
     HexPackPart.sets['EdgeCR2BF'], HexPackPart.sets['EdgeCR3BF'], HexPackPart.sets['EdgeCR4BF'],
     HexPackPart.sets['EdgeCRBF'], HexPackPart.sets['EdgeHI2BF'], HexPackPart.sets['EdgeHI4BF'],
     HexPackPart.sets['EdgeHIBF'], HexPackPart.sets['EdgeHI3BF']))
    r = HexPackPart.sets['backside_flip11'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end1Edges=r, ratio=bias_ratio, number=no_of_seed, constraint=FINER)
    HexPackPart.SetByBoolean(name='hexagon_backface_flip11', sets=(HexPackPart.sets['EdgeQN2BF'],
     HexPackPart.sets['EdgeQN3BF'], HexPackPart.sets['EdgeQN4BF'], HexPackPart.sets['EdgeQNBF'],
     HexPackPart.sets['EdgeORBF'], HexPackPart.sets['EdgeOR4BF'], HexPackPart.sets['EdgeOR3BF'],
     HexPackPart.sets['EdgeOR2BF']))
    r = HexPackPart.sets['hexagon_backface_flip11'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end1Edges=r, ratio=bias_ratio, number=no_of_seed, constraint=FINER)
    HexPackPart.SetByBoolean(name='hexagon_frontface_flip11', sets=(HexPackPart.sets['EdgeOR4'],
     HexPackPart.sets['EdgeOR3'], HexPackPart.sets['EdgeOR2'], HexPackPart.sets['EdgeOR'], HexPackPart.sets['EdgeQO'],
     HexPackPart.sets['EdgeQO2'], HexPackPart.sets['EdgeQO3'], HexPackPart.sets['EdgeQO4']))
    r = HexPackPart.sets['hexagon_frontface_flip11'].edges
    HexPackPart.seedEdgeByBias(biasMethod=SINGLE, end1Edges=r, ratio=bias_ratio, number=no_of_seed, constraint=FINER)
    if interface_ratio != 0:
        r = HexPackPart.sets['seed_by_number_edges'].edges
        HexPackPart.seedEdgeByNumber(edges=r, number=n_rad, constraint=FINER)
    elif interface_ratio == 0:
        r = HexPackPart.sets['seed_by_number_edges'].edges
        HexPackPart.seedEdgeByNumber(edges=r, number=n_rad - 1 if n_rad > 1 else n_rad, constraint=FINER)
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, w / 2, h / 2))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Corner1')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, w / 2, -(h / 2)))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Corner2')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, -(w / 2), h / 2))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Corner3')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, -(w / 2), -(h / 2)))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Corner4')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, w / 2, 0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Midedge1')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, -(w / 2), 0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Midedge2')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, 0, h / 2))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Midedge3')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, 0, -(h / 2)))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='Midedge4')
    ed20 = HexPackPart.edges
    edgeOY = ed20.findAt((d / 2, 0, 0))
    edgeYOindex = edgeOY.index
    HexPackPart.Set(edges=ed20[edgeYOindex:edgeYOindex + 1], name='origin edge')
    HexPackPart.SetByBoolean(name='depth_bias', sets=(HexPackPart.sets['origin edge'],
     HexPackPart.sets['Midedge1'], HexPackPart.sets['Midedge2'], HexPackPart.sets['Midedge3'],
     HexPackPart.sets['Midedge4'], HexPackPart.sets['Corner1'], HexPackPart.sets['Corner2'],
     HexPackPart.sets['Corner3'], HexPackPart.sets['Corner4']))
    r = HexPackPart.sets['depth_bias'].edges
    HexPackPart.seedEdgeByNumber(edges=r, number=n_depth, constraint=FINER)
    elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, secondOrderAccuracy=OFF, distortionControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
    r = regionToolset.Region(cells=HexPackPart.sets['matrix_zone'].cells)
    HexPackPart.setElementType(regions=r, elemTypes=(elemType1, elemType2,
     elemType3))
    r = regionToolset.Region(cells=HexPackPart.sets['all_fiber_zone'].cells)
    HexPackPart.setElementType(regions=r, elemTypes=(elemType1, elemType2,
     elemType3))
    if interface_ratio != 0:
        elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD)
        elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
        elemType3 = mesh.ElemType(elemCode=UNKNOWN_TET, elemLibrary=STANDARD)
    elif interface_ratio == 0:
        elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, secondOrderAccuracy=OFF, distortionControl=DEFAULT)
        elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
        elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
    if not interface_ratio == 0:
        r = regionToolset.Region(cells=HexPackPart.sets['cohesive_zone'].cells)
        HexPackPart.setElementType(regions=r, elemTypes=(elemType1, elemType2,
         elemType3))
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBCBF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDEBF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEFBF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGHBF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), -(5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC2BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), -(5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE2BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), -(h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF2BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), -(h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH2BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), -(5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC3BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), -(5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE3BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), -(h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF3BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), -(h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH3BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC4BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE4BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF4BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((d, -(w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH4BF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), -(5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC2')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), -(5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE2')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180), -(h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF2')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180), -(h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH2')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), -(5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC3')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), -(5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE3')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), -(h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF3')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), -(h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180))))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH3')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialBC4')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialDE4')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(w / 2 - 5.0 / 6.0 * radius * math.sin(30 * math.pi / 180)), h / 2 - 5.0 / 6.0 * radius * math.cos(30 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialEF4')
    ed21 = HexPackPart.edges
    edgeFL = ed21.findAt((0.0, -(w / 2 - 5.0 / 6.0 * radius * math.sin(90 * math.pi / 180)), h / 2 - 5.0 / 6.0 * radius * math.cos(90 * math.pi / 180)))
    edgeFKindex = edgeFL.index
    HexPackPart.Set(edges=ed21[edgeFKindex:edgeFKindex + 1], name='RadialGH4')
    HexPackPart.SetByBoolean(name='seed_by_size', sets=(HexPackPart.sets['RadialBC'],
     HexPackPart.sets['RadialBC2'], HexPackPart.sets['RadialBC2BF'], HexPackPart.sets['RadialBC3'],
     HexPackPart.sets['RadialBC3BF'], HexPackPart.sets['RadialBC4'], HexPackPart.sets['RadialBC4BF'],
     HexPackPart.sets['RadialBCBF'], HexPackPart.sets['RadialDE'], HexPackPart.sets['RadialDE2'],
     HexPackPart.sets['RadialDE2BF'], HexPackPart.sets['RadialDE3'], HexPackPart.sets['RadialDE3BF'],
     HexPackPart.sets['RadialDE4'], HexPackPart.sets['RadialDE4BF'], HexPackPart.sets['RadialDEBF'],
     HexPackPart.sets['RadialEF'], HexPackPart.sets['RadialEF2'], HexPackPart.sets['RadialEF2BF'],
     HexPackPart.sets['RadialEF3'], HexPackPart.sets['RadialEF3BF'], HexPackPart.sets['RadialEF4'],
     HexPackPart.sets['RadialEF4BF'], HexPackPart.sets['RadialEFBF'], HexPackPart.sets['RadialGH'],
     HexPackPart.sets['RadialGH2'], HexPackPart.sets['RadialGH2BF'], HexPackPart.sets['RadialGH3'],
     HexPackPart.sets['RadialGH3BF'], HexPackPart.sets['RadialGH4'], HexPackPart.sets['RadialGH4BF'],
     HexPackPart.sets['RadialGHBF']))
    r = HexPackPart.sets['seed_by_size'].edges
    HexPackPart.seedEdgeBySize(edges=r, size=(l1 + ln) / 2, deviationFactor=0.1, minSizeFactor=0.1, constraint=FINER)
    HexPackPart.generateMesh()
    HexPack.rootAssembly.Instance(name='HexPackPart-1', part=HexPackPart, dependent=ON)
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=('Corner1',
     'Corner2', 'Corner3', 'Corner4', 'EdgeAB', 'EdgeAB2', 'EdgeAB2BF',
     'EdgeAB3', 'EdgeAB3BF', 'EdgeAB4', 'EdgeAB4BF', 'EdgeABBF', 'EdgeBC',
     'EdgeBC2', 'EdgeBC2BF', 'EdgeBC3', 'EdgeBC3BF', 'EdgeBC4', 'EdgeBC4BF',
     'EdgeBCBF', 'EdgeCR', 'EdgeCR2', 'EdgeCR2BF', 'EdgeCR3', 'EdgeCR3BF',
     'EdgeCR4', 'EdgeCR4BF', 'EdgeCRBF', 'EdgeDE', 'EdgeDE2', 'EdgeDE2BF',
     'EdgeDE3', 'EdgeDE3BF', 'EdgeDE4', 'EdgeDE4BF', 'EdgeDEBF', 'EdgeEF',
     'EdgeEF2', 'EdgeEF2BF', 'EdgeEF3', 'EdgeEF3BF', 'EdgeEF4', 'EdgeEF4BF',
     'EdgeEFBF', 'EdgeFS', 'EdgeFS2', 'EdgeFS2BF', 'EdgeFS3', 'EdgeFS3BF',
     'EdgeFS4', 'EdgeFS4BF', 'EdgeFSBF', 'EdgeGH', 'EdgeGH2', 'EdgeGH2BF',
     'EdgeGH3', 'EdgeGH3BF', 'EdgeGH4', 'EdgeGH4BF', 'EdgeGHBF', 'EdgeHI',
     'EdgeHI2', 'EdgeHI2BF', 'EdgeHI3', 'EdgeHI3BF', 'EdgeHI4', 'EdgeHI4BF',
     'EdgeHIBF', 'EdgeIU', 'EdgeIU2', 'EdgeIU2BF', 'EdgeIU3', 'EdgeIU3BF',
     'EdgeIU4', 'EdgeIU4BF', 'EdgeIUBF', 'EdgeJK', 'EdgeJK2', 'EdgeJK2BF',
     'EdgeJK3', 'EdgeJK3BF', 'EdgeJK4', 'EdgeJK4BF', 'EdgeJKBF', 'EdgeKL',
     'EdgeKL2', 'EdgeKL2BF', 'EdgeKL3', 'EdgeKL3BF', 'EdgeKL4', 'EdgeKL4BF',
     'EdgeKLBF', 'EdgeLT', 'EdgeLT2', 'EdgeLT2BF', 'EdgeLT3', 'EdgeLT3BF',
     'EdgeLT4', 'EdgeLT4BF', 'EdgeLTBF', 'EdgeOL', 'EdgeOL2', 'EdgeOL2BF',
     'EdgeOL3', 'EdgeOL3BF', 'EdgeOL4', 'EdgeOL4BF', 'EdgeOLBF', 'EdgeOM',
     'EdgeOM2', 'EdgeOM2BF', 'EdgeOM3', 'EdgeOM3BF', 'EdgeOM4', 'EdgeOM4BF',
     'EdgeOMBF', 'EdgeON', 'EdgeON2', 'EdgeON2BF', 'EdgeON3', 'EdgeON3BF',
     'EdgeON4', 'EdgeON4BF', 'EdgeONBF', 'EdgeOO', 'EdgeOO2', 'EdgeOO2BF',
     'EdgeOO3', 'EdgeOO3BF', 'EdgeOO4', 'EdgeOO4BF', 'EdgeOOBF', 'EdgeOR',
     'EdgeOR2', 'EdgeOR2BF', 'EdgeOR3', 'EdgeOR3BF', 'EdgeOR4', 'EdgeOR4BF',
     'EdgeORBF', 'EdgeOV', 'EdgeOV2', 'EdgeOV2BF', 'EdgeOV3', 'EdgeOV3BF',
     'EdgeOV4', 'EdgeOV4BF', 'EdgeOVBF', 'EdgeOY', 'EdgeOY2', 'EdgeOY2BF',
     'EdgeOY3', 'EdgeOY3BF', 'EdgeOY4', 'EdgeOY4BF', 'EdgeOYBF', 'EdgeQB',
     'EdgeQB2', 'EdgeQB2BF', 'EdgeQB3', 'EdgeQB3BF', 'EdgeQB4', 'EdgeQB4BF',
     'EdgeQBBF', 'EdgeQC', 'EdgeQC2', 'EdgeQC2BF', 'EdgeQC3', 'EdgeQC3BF',
     'EdgeQC4', 'EdgeQC4BF', 'EdgeQCBF', 'EdgeQF', 'EdgeQF2', 'EdgeQF2BF',
     'EdgeQF3', 'EdgeQF3BF', 'EdgeQF4', 'EdgeQF4BF', 'EdgeQFBF', 'EdgeQG',
     'EdgeQG2', 'EdgeQG2BF', 'EdgeQG3', 'EdgeQG3BF', 'EdgeQG4', 'EdgeQG4BF',
     'EdgeQGBF', 'EdgeQM', 'EdgeQM2', 'EdgeQM2BF', 'EdgeQM3', 'EdgeQM3BF',
     'EdgeQM4', 'EdgeQM4BF', 'EdgeQMBF', 'EdgeQN', 'EdgeQN2', 'EdgeQN2BF',
     'EdgeQN3', 'EdgeQN3BF', 'EdgeQN4', 'EdgeQN4BF', 'EdgeQNBF', 'EdgeQO',
     'EdgeQO2', 'EdgeQO2BF', 'EdgeQO3', 'EdgeQO3BF', 'EdgeQO4', 'EdgeQO4BF',
     'EdgeQOBF', 'EdgeQP', 'EdgeQP2', 'EdgeQP2BF', 'EdgeQP3', 'EdgeQP3BF',
     'EdgeQP4', 'EdgeQP4BF', 'EdgeQPBF', 'EdgeRM', 'EdgeRM2', 'EdgeRM2BF',
     'EdgeRM3', 'EdgeRM3BF', 'EdgeRM4', 'EdgeRM4BF', 'EdgeRMBF', 'EdgeRV',
     'EdgeRV2', 'EdgeRV2BF', 'EdgeRV3', 'EdgeRV3BF', 'EdgeRV4', 'EdgeRV4BF',
     'EdgeRVBF', 'EdgeVG', 'EdgeVG2', 'EdgeVG2BF', 'EdgeVG3', 'EdgeVG3BF',
     'EdgeVG4', 'EdgeVG4BF', 'EdgeVGBF', 'Egde OW', 'Egde OW2', 'Egde OW2BF',
     'Egde OW3', 'Egde OW3BF', 'Egde OW4', 'Egde OW4BF', 'Egde OWBF', 'Egde OX',
     'Egde OX2', 'Egde OX2BF', 'Egde OX3', 'Egde OX3BF', 'Egde OX4',
     'Egde OX4BF', 'Egde OXBF'))
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=(
     'Midedge1', 'Midedge2', 'Midedge3', 'Midedge4'))
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=(
     'RadialKLBF', 'RadialAB', 'RadialAB2', 'RadialAB2BF', 'RadialAB3',
     'RadialAB3BF', 'RadialAB4', 'RadialAB4BF', 'RadialABBF', 'RadialBC',
     'RadialBC2', 'RadialBC2BF', 'RadialBC3', 'RadialBC3BF', 'RadialBC4',
     'RadialBC4BF', 'RadialBCBF', 'RadialCD', 'RadialCD2', 'RadialCD2BF',
     'RadialCD3', 'RadialCD3BF', 'RadialCD4', 'RadialCD4BF', 'RadialCDBF',
     'RadialDE', 'RadialDE2', 'RadialDE2BF', 'RadialDE3', 'RadialDE3BF',
     'RadialDE4', 'RadialDE4BF', 'RadialDEBF', 'RadialEF', 'RadialEF2',
     'RadialEF2BF', 'RadialEF3', 'RadialEF3BF', 'RadialEF4', 'RadialEF4BF',
     'RadialEFBF', 'RadialFG', 'RadialFG2', 'RadialFG2BF', 'RadialFG3',
     'RadialFG3BF', 'RadialFG4', 'RadialFG4BF', 'RadialFGBF', 'RadialGH',
     'RadialGH2', 'RadialGH2BF', 'RadialGH3', 'RadialGH3BF', 'RadialGH4',
     'RadialGH4BF', 'RadialGHBF', 'RadialKL', 'RadialKL2', 'RadialKL2BF',
     'RadialKL3', 'RadialKL3BFBF', 'RadialKL4', 'RadialKL4BF'))
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=('Set-303',
     'cohesive_zone_radial_edges'))
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=(
     'origin edge', 'hexagon_frontface_flip11', 'hexagon_backface_flip11',
     'hex_edges_frontflip', 'frontface_rightflip', 'frontface_flipset11',
     'frontface_flipSet', 'front_face_flipright', 'final_frontFlip'))
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=(
     'backside_flip11', 'backface_flipSet'))
    del mdb.models[RVE_modelName].parts['HexPackPart'].sets['front_face_all']
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=(
     'all_fiber_coh_zone', 'center_fiber_coh_zone', 'coh_center_ring',
     'coh_corner1_ring', 'coh_corner2_ring', 'coh_corner3_ring',
     'coh_corner4_ring', 'cohesive_zoneA_cell', 'cohesive_zoneB_cell',
     'corner1_fiber', 'corner1_fiber_coh_zone', 'corner2_fiber',
     'corner2_fiber_coh_zone', 'corner3_fiber', 'corner3_fiber_coh_zone',
     'corner4_fiber', 'corner4_fiber_coh_zone', 'fiber_cell_A', 'middle_cell_1',
     'middle_cell_2'))
    mdb.models[RVE_modelName].parts['HexPackPart'].deleteSets(setNames=(
     'all_cells', 'all_matrix_coh_zone_edges', 'center_fiber', 'depth_bias',
     'seed_by_bias', 'seed_by_bias_number', 'seed_by_min_max_flip',
     'seed_by_min_max_size', 'seed_by_number_edges', 'seed_by_size'))
    return


