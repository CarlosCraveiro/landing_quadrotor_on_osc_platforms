import Pkg; Pkg.activate(@__DIR__);
    Pkg.add("LinearAlgebra");
    Pkg.add("Test");
    Pkg.add("SparseArrays");
    Pkg.add("ForwardDiff");
    Pkg.add("ControlSystems");
    Pkg.add("OSQP"); # Our solver
    Pkg.add("BlockDiagonals");
    
    Pkg.add("Plots"); # To allow Ploting

    # To allow 3D Visualization
    Pkg.add("MeshCat");
    Pkg.build("MeshCat");
    Pkg.add("TrajOptPlots");
    Pkg.add("RobotZoo");
    Pkg.add("StaticArrays");

#    Pkg.add("Metaheuristics"); To a future work on optimizing Q and R choice to reduce land time
Pkg.instantiate()