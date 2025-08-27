{
  description = "Sphinx documentation environment with sphinx-tags";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };

      python = pkgs.python313;

      sphinx-tags = python.pkgs.buildPythonPackage rec {
        pname = "sphinx-tags";
        version = "0.4";

        src = pkgs.fetchPypi {
          pname = "sphinx_tags";
          inherit version;
          sha256 = "MGUhm6z0dWfHBvIjfVZlsi86UWc2e0xFaLzaQ3GlNZ0=";
        };

        pyproject = true;
        build-system = [ python.pkgs.flit-core ];

        propagatedBuildInputs = with python.pkgs; [
          sphinx
        ];

        doCheck = false;
      };

      sphinx-collapse = python.pkgs.buildPythonPackage rec {
        pname = "sphinx_collapse";
        version = "0.1.3";

        src = pkgs.fetchPypi {
          inherit version pname;
          sha256 = "yuFB5vA+zVLtJGowWmnhsNXQXmzfP+gD1A1YOtatiVo=";
        };

        pyproject = true;
        build-system = [ python.pkgs.flit-core ];

        propagatedBuildInputs = with python.pkgs; [
          sphinx
        ];

        doCheck = false;
      };

      nuttx-doc-py-env = python.withPackages (
        ps: with ps; [
          sphinx
          sphinx_rtd_theme
          myst-parser
          sphinx-tabs
          sphinx-autobuild
          sphinx-copybutton
          sphinx-togglebutton
          pytz
          importlib-metadata
          sphinx-design
          sphinx-tags
          sphinx-collapse
        ]
      );
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [ nuttx-doc-py-env ];
        shellHook = ''
          echo "Welcome to NuttX documentation devShell"
        '';
      };
    };
}
