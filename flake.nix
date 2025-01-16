{
  description = "Group 6 MASLAB";
  
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    # nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        adafruit-blinka = with pkgs.python3Packages; buildPythonPackage rec {
          pname = "Adafruit-Blinka";
          version = "8.38.0";

          build-system = [setuptools-scm];
          format = "setuptools";

          src = fetchPypi {
            inherit pname version;
            hash = "sha256-uxBKWYohMFO/ViTtMFV57wIe/uYz3+kf0ddMb/Kr2+M=";
          };
        };

        maslab-lib-src = fetchGit {
          url = "https://github.com/MASLAB/maslab-lib";
          ref = "main";
          rev = "6788cd2f1fcba8b5dc430cde1abeae300d4caa49";
        };

        maslab-lib = pkgs.python3Packages.buildPythonPackage {
          name = "maslab-lib";

          build-system = [pkgs.python3Packages.setuptools];

          dependencies = with pkgs.python3Packages; [
            pyyaml
            numpy
            pyserial
            adafruit-blinka
          ];

          src = maslab-lib-src;
          format = "pyproject";
        };
        
      in {
        devShells.default = pkgs.mkShell {
          packages =
            with pkgs; [
              colcon
              # ament-cmake-python

              cargo
              rustc
              rustfmt
              rust-analyzer
              rustPackages.clippy
              python312Packages.setuptools
              # for python hardware package
              python314
              pyright
              black
              # for rust hardware package
              clang
              llvmPackages.libclang
              udev
              (with rosPackages.jazzy; buildEnv {
                paths = [
                  ros-core
                  ament-cmake
                  ament-cmake-core
                  ament-cmake-python
                  python-cmake-module
                  pkg-config
                  std-msgs
                  rosidl-generator-py
                  rosidl-core-generators
                ];
              })
            ];

          buildInputs = [maslab-lib];

          # bindgen needs to find libclang
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
  
        };
      });

  # nixConfig = {
  #   extra-substituters = [ "https://ros.cachix.org" ];
  #   extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  # };

}
