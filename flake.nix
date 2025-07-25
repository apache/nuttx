{
  description = "NuttX : devShell flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            # Build tools
            pkgs.cmake
            pkgs.ninja
            pkgs.gnumake
            pkgs.clang-tools

            # ARM toolchain
            pkgs.gcc-arm-embedded

            # NuttX dependencies
            pkgs.automake
            pkgs.bison
            pkgs.flex
            pkgs.genromfs
            pkgs.gettext
            pkgs.gperf
            pkgs.kconfig-frontends
            pkgs.libelf
            pkgs.expat.dev
            pkgs.gmp.dev
            pkgs.isl
            pkgs.libmpc
            pkgs.mpfr.dev
            pkgs.ncurses.dev
            pkgs.zlib
            pkgs.python313Packages.kconfiglib

            # NuttX Documentation
            pkgs.sphinx
            pkgs.python313Packages.sphinx_rtd_theme
            pkgs.python313Packages.myst-parser
            pkgs.python313Packages.sphinx-tabs
            pkgs.python313Packages.sphinx-autobuild
            pkgs.python313Packages.sphinx-copybutton
            pkgs.python313Packages.sphinx-togglebutton
            pkgs.python313Packages.pytz
            pkgs.python313Packages.importlib-metadata
            pkgs.python313Packages.sphinx-design
          ];
          shellHook = ''
            export CMAKE_EXPORT_COMPILE_COMMANDS=ON
            echo "Welcome to NuttX devShell"
          '';
        };
      }
    );
}
