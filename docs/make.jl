using MacroEnergySolvers
using Documenter

DocMeta.setdocmeta!(MacroEnergySolvers, :DocTestSetup, :(using MacroEnergySolvers); recursive=true)

# Build documentation.
# ====================
makedocs(;
    modules=[MacroEnergySolvers],
    authors="F. Pecci <filippo.pecci@cmcc.it>, L. Bonaldo <bonaldo.luca12@gmail.com>, J. D. Jenkins <jessejenkins@princeton.edu>",
    sitename="MacroEnergySolvers",
    format = Documenter.HTML(; prettyurls = get(ENV, "CI", "false") == "true"),
    pages=[
        "Getting Started" => "index.md",
    ],
)

# Deploy built documentation.
# ===========================
deploydocs(;
    repo="github.com/macroenergy/MacroEnergySolvers.jl.git",
    target = "build",
    branch = "gh-pages",
    devbranch = "main",
    devurl = "dev",
    push_preview=true,
)
