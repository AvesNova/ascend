include("pga_helper.jl")

@define_clifford_algebra_helpers PGA2D pga Motor 1 5 6 7
@define_clifford_algebra_helpers PGA2D pga Line 2 3 4
@define_clifford_algebra_helpers PGA2D pga Rotor 1 5
@define_clifford_algebra_helpers PGA2D pga Translator 1 6 7
@define_clifford_algebra_helpers PGA2D pga Point 5 6 7
@define_clifford_algebra_helpers PGA2D pga Direction 6 7
@define_clifford_algebra_helpers PGA2D pga Sudo_Scalar 8
@define_clifford_algebra_helpers PGA2D pga Multivector 1 2 3 4 5 6 7 8
