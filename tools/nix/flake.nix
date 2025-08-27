{
  description = "NuttX : devShell flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    documentation.url = "path:./doc/";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      documentation,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
      in
      {
        # Default devShell
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
          ];
          shellHook = ''
            export CMAKE_EXPORT_COMPILE_COMMANDS=ON
            echo "Welcome to NuttX devShell"
          '';
        };

        # Documentation devShell
        devShells.docs = documentation.devShells.${system}.default;
      }
    );
}
