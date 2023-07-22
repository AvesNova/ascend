using Documenter
using AvesAscend

makedocs(
    sitename = "AvesAscend",
    # format = Documenter.HTML(),
    modules = [AvesAscend]
)

deploydocs(
    repo = "https://github.com/Vizia128/Aves_Ascend",
    versions = nothing,
)
