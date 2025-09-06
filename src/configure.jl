import Pkg; Pkg.activate(@__DIR__);

 #   Pkg.add("CoordinateTransformations");
 #   Pkg.add("Rotations");
 #   Pkg.add("Colors");

    Pkg.add("LinearAlgebra");
    Pkg.add("Test");
    Pkg.add("SparseArrays");
    Pkg.add("ForwardDiff");
    Pkg.add("ControlSystems");
    Pkg.add("OSQP");
    Pkg.add("BlockDiagonals");
    
    Pkg.add("Plots");

#    Pkg.add("RobotDynamics");
#    Pkg.add("GeometryBasics");
    Pkg.add("MeshCat");
    Pkg.build("MeshCat");
    Pkg.add("TrajOptPlots");
    Pkg.add("RobotZoo");
    Pkg.add("StaticArrays");
    Pkg.add("Metaheuristics");
Pkg.instantiate()