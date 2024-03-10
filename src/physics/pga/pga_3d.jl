include("pga_helper.jl")

@define_clifford_algebra_helpers PGA3D pga Motor 1 6 7 8 9 10 11 16
@define_clifford_algebra_helpers PGA3D pga Line 6 7 8 9 10 11
@define_clifford_algebra_helpers PGA3D pga Rotor 1 6 7 8
@define_clifford_algebra_helpers PGA3D pga Translator 1 9 10 11
@define_clifford_algebra_helpers PGA3D pga Point 2 3 4 5
@define_clifford_algebra_helpers PGA3D pga Direction 2 3 4
@define_clifford_algebra_helpers PGA3D pga Plane 12 13 14 15
@define_clifford_algebra_helpers PGA3D pga Sudo_Scalar 16
@define_clifford_algebra_helpers PGA3D pga Multivector 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16