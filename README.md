# MacroEnergySystemsDecomposition.jl

Collection of decomposition algorithms to solve macro-energy system models generate by open-source packages MACRO, GenX, DOLPHYN.

In order to use these methods, the selected macro-energy system modelling package needs to have functions to:

1. Separate inputs into sub-periods, and generate a dictionary mapping each sub-period index to the corresponding set of inputs.
2. Define linking variables and obtain their string names from JuMP.
3. Generate a planning problem.
4. Generate an operational sub-problem given a set of inputs corresponding to one sub-period.

To install GenX as a package, type:
`]add GenX`

To install MACRO as a package, type:
`]add git@github.com:macroenergy/Macro.git #core_structure`

To install Dolphyn as a package, type:
`]add Dolphyn`
