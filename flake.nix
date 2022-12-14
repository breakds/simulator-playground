{
  description = "Playground and evaluation of various robotics/physics simulator";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-22.11";

    utils.url = "github:numtide/flake-utils";

    ml-pkgs.url = "github:nixvital/ml-pkgs";
    ml-pkgs.inputs.nixpkgs.follows = "nixpkgs";
    ml-pkgs.inputs.utils.follows = "utils";
  };

  outputs = { self, nixpkgs, utils, ... }@inputs: utils.lib.eachSystem [
    "x86_64-linux"
  ] (system: let
    pkgs = import nixpkgs {
      inherit system;
      config.allowUnfree = true;
      overlays = [
        inputs.ml-pkgs.overlays.simulators
      ];
    };

  in rec {
    devShells = {
      default = pkgs.mkShell rec {
        name = "simulator-playground";

        packages = let pythonEnv = pkgs.python3.withPackages (pyPkgs: with pyPkgs; [
          numpy
          matplotlib

          pybullet

          # Jupyter Lab
          jupyterlab
          ipywidgets
          rich
        ]); in [
          pythonEnv
          # Dev Tools
          pkgs.nodePackages.pyright
          pkgs.pre-commit
        ];

        shellHook = ''
          export PS1="$(echo -e '\uf3e2') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
          export PYTHONPATH="$(pwd):$PYTHONPATH"
        '';
      };
    };
  });
}
