# SCRIPT TO RUN THE DECADAL SIMULATION USING THE NEW PFTOOLS

import sys
import os
from datetime import datetime
from parflow.tools import Run
from parflow.tools.fs import mkdir, cp, get_absolute_path, exists
from parflow.tools.settings import set_working_directory


#-----------------------------------------------------------------------------
#Get initial inputs
#-----------------------------------------------------------------------------

runname = 'Lower_Miss'

LM = Run(runname, __file__)
LM.FileVersion = 4

#-----------------------------------------------------------------------------
# Set Processor topology
#-----------------------------------------------------------------------------

LM.Process.Topology.P = 15
LM.Process.Topology.Q = 15
LM.Process.Topology.R = 1

nproc = LM.Process.Topology.P * LM.Process.Topology.Q * LM.Process.Topology.R

#-----------------------------------------------------------------------------
# Make a directory for the simulation and copy inputs into it
#-----------------------------------------------------------------------------

set_working_directory(get_absolute_path('.'))

# ParFlow Inputs
cp('../inputs/Lower_Miss.slopex.pfb')
cp('../inputs/Lower_Miss.slopey.pfb')
# cp('../inputs/model_inputs/LM.0.Final1km_mannings_rv50_original_values.pfb')
# cp('../inputs/model_inputs/1km_LM_PME.pfb')
# cp('../inputs/model_inputs/LM.0.Final1km.Subsurface.pfb')
# cp('../inputs/model_inputs/Shangguan_200m_FBZ.pfb')
cp('../inputs/Lower_Miss.pfsol')

#-----------------------------------------------------------------------------
# Initial pressure
#-----------------------------------------------------------------------------

curr_step = 0
#ip = runname+'.out.press.%05d.pfb' %(curr_step)
#ip = runname+'.init.press.pfb'

#-----------------------------------------------------------------------------
# Computational Grid
#-----------------------------------------------------------------------------

LM.ComputationalGrid.Lower.X = 0.0
LM.ComputationalGrid.Lower.Y = 0.0
LM.ComputationalGrid.Lower.Z = 0.0

LM.ComputationalGrid.DX = 1000.0
LM.ComputationalGrid.DY = 1000.0
LM.ComputationalGrid.DZ = 200.0

LM.ComputationalGrid.NX = 625
LM.ComputationalGrid.NY = 900
LM.ComputationalGrid.NZ = 10

#-----------------------------------------------------------------------------
# Names of the GeomInputs
#-----------------------------------------------------------------------------

LM.GeomInput.Names = "domaininput indi_input"
LM.GeomInput.Names = "domaininput"

#-----------------------------------------------------------------------------
# Domain Geometry Input
#-----------------------------------------------------------------------------

LM.GeomInput.domaininput.InputType = 'SolidFile'
LM.GeomInput.domaininput.GeomNames = 'domain'
LM.GeomInput.domaininput.FileName = 'Lower_Miss.pfsol'

#-----------------------------------------------------------------------------
# Domain Geometry
#-----------------------------------------------------------------------------

LM.Geom.domain.Patches = "ocean land top lake sink bottom"

#-----------------------------------------------------------------------------
# Indicator Geometry Input
#-----------------------------------------------------------------------------


#--------------------------------------------
# variable dz assignments
#------------------------------------------
LM.Solver.Nonlinear.VariableDz = True
LM.dzScale.GeomNames = 'domain'
LM.dzScale.Type = 'nzList'
LM.dzScale.nzListNumber = 10

# 10 layers, starts at 0 for the bottom to 9 at the top
# note this is opposite Noah/WRF
# layers are 0.1 m, 0.3 m, 0.6 m, 1.0 m, 5.0 m, 10.0 m, 25.0 m, 50.0 m, 100.0m, 200.0 m
# 200 m * 1.0 = 200 m
LM.Cell._0.dzScale.Value = 1.0
# 200 m * .5 = 100 m 
LM.Cell._1.dzScale.Value = 0.5
# 200 m * .25 = 50 m 
LM.Cell._2.dzScale.Value = 0.25
# 200 m * 0.125 = 25 m 
LM.Cell._3.dzScale.Value = 0.125
# 200 m * 0.05 = 10 m 
LM.Cell._4.dzScale.Value = 0.05
# 200 m * .025 = 5 m 
LM.Cell._5.dzScale.Value = 0.025
# 200 m * .005 = 1 m 
LM.Cell._6.dzScale.Value = 0.005
# 200 m * 0.003 = 0.6 m 
LM.Cell._7.dzScale.Value = 0.003
# 200 m * 0.0015 = 0.3 m 
LM.Cell._8.dzScale.Value = 0.0015
# 200 m * 0.0005 = 0.1 m = 10 cm which is default top Noah layer
LM.Cell._9.dzScale.Value = 0.0005

#------------------------------------------------------------------------------
# Flow Barrier defined by Shangguan Depth to Bedrock
#--------------------------------------------------------------

LM.Solver.Nonlinear.FlowBarrierZ = False
# LM.FBz.Type = 'PFBFile'
# LM.Geom.domain.FBz.FileName = 'Shangguan_200m_FBZ.pfb'
# LM.dist('Shangguan_200m_FBZ.pfb')

#-----------------------------------------------------------------------------
# Permeability (values in m/hr)
#-----------------------------------------------------------------------------

LM.Geom.Perm.Names = 'domain'

LM.Geom.domain.Perm.Type = 'Constant'
LM.Geom.domain.Perm.Value = 0.02
LM.Geom.domain.Perm.Value = 2e-5

LM.Perm.TensorType = 'TensorByGeom'
LM.Geom.Perm.TensorByGeom.Names = 'domain b1 b2 g1 g2 g4 g5 g6 g7'
LM.Geom.Perm.TensorByGeom.Names = 'domain'

LM.Geom.domain.Perm.TensorValX = 1.0
LM.Geom.domain.Perm.TensorValY = 1.0
LM.Geom.domain.Perm.TensorValZ = 1.0

#-----------------------------------------------------------------------------
# Specific Storage
#-----------------------------------------------------------------------------

LM.SpecificStorage.Type = 'Constant'
LM.SpecificStorage.GeomNames = 'domain'
LM.Geom.domain.SpecificStorage.Value = 1.0e-4

#-----------------------------------------------------------------------------
# Phases
#-----------------------------------------------------------------------------

LM.Phase.Names = 'water'
LM.Phase.water.Density.Type = 'Constant'
LM.Phase.water.Density.Value = 1.0
LM.Phase.water.Viscosity.Type = 'Constant'
LM.Phase.water.Viscosity.Value = 1.0

#-----------------------------------------------------------------------------
# Contaminants
#-----------------------------------------------------------------------------

LM.Contaminants.Names = ''

#-----------------------------------------------------------------------------
# Gravity
#-----------------------------------------------------------------------------

LM.Gravity = 1.0

#-----------------------------------------------------------------------------
# Timing (time units is set by units of permeability)
#-----------------------------------------------------------------------------

LM.TimingInfo.BaseUnit = 1.0
LM.TimingInfo.StartCount = curr_step
LM.TimingInfo.StartTime = curr_step
LM.TimingInfo.StopTime = 60
LM.TimingInfo.DumpInterval = 1.0
LM.TimeStep.Type = 'Constant'
LM.TimeStep.Value = 0.1

# LM.TimeStep.Type = 'Growth'
# LM.TimeStep.InitialStep = 0.005
# LM.TimeStep.GrowthFactor = 1.02
# LM.TimeStep.MaxStep = 100
# LM.TimeStep.MinStep = 0.0001

#-----------------------------------------------------------------------------
# Porosity
#-----------------------------------------------------------------------------

LM.Geom.Porosity.GeomNames = 'domain s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 g1 g2 g3 g4 g5 g6 g7 g8'
LM.Geom.Porosity.GeomNames = 'domain'

LM.Geom.domain.Porosity.Type = 'Constant'
LM.Geom.domain.Porosity.Value = 0.33

#-----------------------------------------------------------------------------
# Domain
#-----------------------------------------------------------------------------

LM.Domain.GeomName = 'domain'

#----------------------------------------------------------------------------
# Mobility
#----------------------------------------------------------------------------

LM.Phase.water.Mobility.Type = 'Constant'
LM.Phase.water.Mobility.Value = 1.0

#-----------------------------------------------------------------------------
# Wells
#-----------------------------------------------------------------------------

LM.Wells.Names = ''

#-----------------------------------------------------------------------------
# Relative Permeability
#-----------------------------------------------------------------------------

LM.Phase.RelPerm.Type = 'VanGenuchten'
LM.Phase.RelPerm.GeomNames = 'domain s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13'
LM.Phase.RelPerm.GeomNames = 'domain'

LM.Geom.domain.RelPerm.Alpha = 0.5
LM.Geom.domain.RelPerm.N = 2.5
LM.Geom.domain.RelPerm.NumSamplePoints = 20000
LM.Geom.domain.RelPerm.MinPressureHead = -500
LM.Geom.domain.RelPerm.InterpolationMethod = 'Linear'

#-----------------------------------------------------------------------------
# Saturation
#-----------------------------------------------------------------------------

LM.Phase.Saturation.Type = 'VanGenuchten'
LM.Phase.Saturation.GeomNames = 'domain s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13'
LM.Phase.Saturation.GeomNames = 'domain'

LM.Geom.domain.Saturation.Alpha = 0.5
LM.Geom.domain.Saturation.N = 2.5
LM.Geom.domain.Saturation.SRes = 0.00001
LM.Geom.domain.Saturation.SSat = 1.0

#-----------------------------------------------------------------------------
# Time Cycles
#-----------------------------------------------------------------------------

LM.Cycle.Names = 'constant rainrec'
LM.Cycle.constant.Names = 'alltime'
LM.Cycle.constant.alltime.Length = 1
LM.Cycle.constant.Repeat = -1

LM.Cycle.rainrec.Names = 'rain rec'
LM.Cycle.rainrec.rain.Length = 10
LM.Cycle.rainrec.rec.Length = 150
LM.Cycle.rainrec.Repeat = -1

#-----------------------------------------------------------------------------
# Boundary Conditions
#-----------------------------------------------------------------------------

LM.BCPressure.PatchNames = LM.Geom.domain.Patches

# LM.Patch.ocean.BCPressure.Type = 'DirEquilRefPatch'
LM.Patch.ocean.BCPressure.Type = 'FluxConst'
LM.Patch.ocean.BCPressure.Cycle = 'constant'
LM.Patch.ocean.BCPressure.RefGeom = 'domain'
LM.Patch.ocean.BCPressure.RefPatch = 'top'
LM.Patch.ocean.BCPressure.alltime.Value = 0.0

# LM.Patch.sink.BCPressure.Type = 'OverlandDiffusive'
LM.Patch.sink.BCPressure.Type = 'OverlandKinematic'
# LM.Patch.sink.BCPressure.Type = 'SeepageFace'
LM.Patch.sink.BCPressure.Cycle = 'constant'
LM.Patch.sink.BCPressure.RefGeom = 'domain'
LM.Patch.sink.BCPressure.RefPatch = 'top'
LM.Patch.sink.BCPressure.alltime.Value = 0.0

# LM.Patch.lake.BCPressure.Type = 'OverlandDiffusive'
LM.Patch.lake.BCPressure.Type = 'OverlandKinematic'
# LM.Patch.lake.BCPressure.Type = 'SeepageFace'
LM.Patch.lake.BCPressure.Cycle = 'constant'
LM.Patch.lake.BCPressure.RefGeom = 'domain'
LM.Patch.lake.BCPressure.RefPatch = 'top'
LM.Patch.lake.BCPressure.alltime.Value = 0.0

LM.Patch.land.BCPressure.Type = 'FluxConst'
LM.Patch.land.BCPressure.Cycle = 'constant'
LM.Patch.land.BCPressure.alltime.Value = 0.0

LM.Patch.bottom.BCPressure.Type = 'FluxConst'
LM.Patch.bottom.BCPressure.Cycle = 'constant'
LM.Patch.bottom.BCPressure.alltime.Value = 0.0

LM.Solver.OverlandKinematic.SeepageOne = 4  ## new key
LM.Solver.OverlandKinematic.SeepageTwo = 5 ## new key

LM.Patch.top.BCPressure.Type = 'OverlandDiffusive'
LM.Patch.top.BCPressure.Type = 'OverlandKinematic'
# LM.Patch.top.BCPressure.Type = 'SeepageFace'
# LM.Patch.top.BCPressure.Cycle = 'constant'
# LM.Patch.top.BCPressure.alltime.Value = 0
LM.Patch.top.BCPressure.Cycle = 'rainrec'
LM.Patch.top.BCPressure.rain.Value = -0.000001
LM.Patch.top.BCPressure.rec.Value = 0.0

# LM.Solver.EvapTransFile = True
# LM.Solver.EvapTrans.FileName = '1km_LM_PME.pfb'
# LM.dist('1km_LM_PME.pfb')

#-----------------------------------------------------------------------------
# Topo slopes in x-direction
#-----------------------------------------------------------------------------

LM.TopoSlopesX.Type = 'PFBFile'
LM.TopoSlopesX.GeomNames = 'domain'
LM.TopoSlopesX.FileName = 'Lower_Miss.slopex.pfb'

#-----------------------------------------------------------------------------
# Topo slopes in y-direction
#-----------------------------------------------------------------------------

LM.TopoSlopesY.Type = 'PFBFile'
LM.TopoSlopesY.GeomNames = 'domain'
LM.TopoSlopesY.FileName = 'Lower_Miss.slopey.pfb'

#-----------------------------------------------------------------------------
# Initial conditions: water pressure
#-----------------------------------------------------------------------------

LM.ICPressure.Type = 'HydroStaticPatch'

# LM.ICPressure.Type = 'PFBFile'
LM.ICPressure.GeomNames = 'domain'
LM.Geom.domain.ICPressure.RefPatch = 'bottom'
# LM.Geom.domain.ICPressure.FileName = ip
# LM.dist(ip)
LM.Geom.domain.ICPressure.RefGeom = 'domain'
LM.Geom.domain.ICPressure.Value = 372.

#-----------------------------------------------------------------------------
# Distribute inputs
#-----------------------------------------------------------------------------
LM.dist('Lower_Miss.slopex.pfb')
LM.dist('Lower_Miss.slopey.pfb')

#LM.dist('LM.0.Final1km.Subsurface.pfb')
#LM.dist(ip)

#-----------------------------------------------------------------------------
# Phase sources:
#-----------------------------------------------------------------------------

LM.PhaseSources.water.Type = 'Constant'
LM.PhaseSources.water.GeomNames = 'domain'
LM.PhaseSources.water.Geom.domain.Value = 0.0

#-----------------------------------------------------------------------------
# Mannings coefficient
#-----------------------------------------------------------------------------

LM.Mannings.Type = 'Constant'
LM.Mannings.GeomNames = 'domain'
LM.Mannings.Geom.domain.Value = 2.4e-6

#-----------------------------------------------------------------------------
# Exact solution specification for error calculations
#-----------------------------------------------------------------------------

LM.KnownSolution = 'NoKnownSolution'

#-----------------------------------------------------------------------------
# Set solver parameters
#-----------------------------------------------------------------------------

LM.Solver = 'Richards'
LM.Solver.TerrainFollowingGrid = True
LM.Solver.TerrainFollowingGrid.SlopeUpwindFormulation = 'Upwind'

LM.Solver.MaxIter = 250000
#LM.Solver.Drop = 1E-30
#LM.Solver.AbsTol = 1E-9
LM.Solver.MaxConvergenceFailures = 5
LM.Solver.Nonlinear.MaxIter = 250
LM.Solver.Nonlinear.ResidualTol = 1e-2
LM.Solver.OverlandDiffusive.Epsilon = 0.1

LM.Solver.PrintTop = True
## new solver settings for Terrain Following Grid
LM.Solver.Nonlinear.EtaChoice = 'EtaConstant'
LM.Solver.Nonlinear.EtaValue = 0.01
LM.Solver.Nonlinear.UseJacobian = True
LM.Solver.Nonlinear.DerivativeEpsilon = 1e-16
LM.Solver.Nonlinear.StepTol = 1e-15
LM.Solver.Nonlinear.Globalization = 'LineSearch'
LM.Solver.Linear.KrylovDimension = 500
LM.Solver.Linear.MaxRestarts = 8

LM.Solver.Linear.Preconditioner = 'PFMG'
LM.Solver.Linear.Preconditioner.PCMatrixType = 'PFSymmetric'
LM.Solver.Linear.Preconditioner.PFMG.NumPreRelax = 3
LM.Solver.Linear.Preconditioner.PFMG.NumPostRelax = 2

LM.Solver.WriteSiloPressure = False
LM.Solver.PrintSubsurfData = True
LM.Solver.PrintMask = True
LM.Solver.PrintVelocities = False
LM.Solver.PrintSaturation = False
LM.Solver.PrintPressure = True
#Writing output (no binary except Pressure, all silo):
LM.Solver.PrintSubsurfData = True
#pfset Solver.PrintLSMSink                        True 
LM.Solver.PrintSaturation = True
LM.Solver.WriteCLMBinary = False
#LM.Solver.PrintCLM = True

LM.Solver.WriteSiloSpecificStorage = False
LM.Solver.WriteSiloMannings = False
LM.Solver.WriteSiloMask = False
LM.Solver.WriteSiloSlopes = False
LM.Solver.WriteSiloSubsurfData = False
LM.Solver.WriteSiloPressure = False
LM.Solver.WriteSiloSaturation = False
LM.Solver.WriteSiloEvapTrans = False
LM.Solver.WriteSiloEvapTransSum = False
LM.Solver.WriteSiloOverlandSum = False
LM.Solver.WriteSiloCLM = False


# pfwritedb $runname

LM.run()
print("ParFlow run complete")
