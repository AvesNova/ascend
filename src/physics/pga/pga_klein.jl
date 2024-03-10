include("pga_helper.jl")

@define_clifford_algebra_helpers Klein pga Motor 5 6 7 8 9 10 11 12
@define_clifford_algebra_helpers Klein pga Line 6 7 8 10 11 12
@define_clifford_algebra_helpers Klein pga Rotor 5 6 7 8
@define_clifford_algebra_helpers Klein pga Translator 5 10 11 12
@define_clifford_algebra_helpers Klein pga Point 1 2 3 4
@define_clifford_algebra_helpers Klein pga Direction 2 3 4
@define_clifford_algebra_helpers Klein pga Plane 13 14 15 16
@define_clifford_algebra_helpers Klein pga Sudo_Scalar 9
@define_clifford_algebra_helpers Klein pga Multivector 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 
