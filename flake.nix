{
  description = "Group 6 MASLAB";
  
  inputs = {
    rospkgs.url = "github:lopsided98/nix-ros-overlay/master";
    # nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
  };

  outputs = { self, rospkgs, nixpkgs }:
    rospkgs.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ rospkgs.overlays.default ];
        };
        ros = import rospkgs {
          inherit system;
          
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
          rev = "1ef6d3d622b486aa20a3329526aee8b03e7714e1";
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
            with pkgs; with ros.rosPackages.jazzy; [
              colcon
              # ament-cmake-python

              cargo
              rustc
              rustfmt
              rust-analyzer
              rustPackages.clippy
              python3Packages.setuptools
              # for python hardware package
              python3
              python3Packages.opencv4
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
                  sensor-msgs
                  rosidl-generator-py
                  rosidl-core-generators
                  # deps
                  v4l2-camera
                  cv-bridge
                  # tools
                  rqt
                  rqt-topic
                  rqt-robot-monitor
                  rqt-console
                  rqt-image-view
                  rviz2
                ];
              })
            ];

          buildInputs = [maslab-lib];

          ROS_DOMAIN_ID = 6;

          # bindgen needs to find libclang
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
  
        };
      });

  # nixConfig = {
  #   extra-substituters = [ "https://ros.cachix.org" ];
  #   extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  # };

}
