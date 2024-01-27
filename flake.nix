{
  description = "Devshell for esp32c3 dev";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    fenix.url = "github:nix-community/fenix";
    crane.url = "github:ipetkov/crane";
    crane.inputs.nixpkgs.follows = "nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, fenix, flake-utils, crane }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ fenix.overlays.default ];
        };
        fx = fenix.packages.${system};
        rust-toolchain = fx.combine [
          fx.latest.cargo
          fx.latest.rustc
          fx.latest.rust-analyzer
          fx.latest.clippy
          fx.latest.rustfmt
          fx.latest.rust-src
          fx.targets.riscv32imc-unknown-none-elf.latest.rust-std
          fx.targets.x86_64-unknown-linux-musl.latest.rust-std
        ];
        craneLib = (crane.mkLib pkgs).overrideToolchain rust-toolchain;
        python-toolchain = pkgs.python3.withPackages (ps: [ps.bleak ps.matplotlib ps.numpy ps.tornado ps.pandas ps.seaborn]);

        commonArgs = {
          src = craneLib.cleanCargoSource (craneLib.path ./.);
          strictDeps = true;
          cargoLock = ./embedded/bradipograph/Cargo.lock;
          cargoToml = ./embedded/bradipograph/Cargo.toml;
          doCheck = false;

          postUnpack = ''
            cd $sourceRoot/embedded/bradipograph
            sourceRoot="."
          '';

          # See https://github.com/ipetkov/crane/issues/444
          extraDummyScript = ''
            rm -rf $out/embedded/bradipograph/src/bin/crane-dummy-*
          '';
        };

        cargoArtifacts = craneLib.buildDepsOnly (commonArgs // {
          pname = "bradipograph-deps";
        });

        firmware = craneLib.buildPackage (commonArgs // {
          inherit cargoArtifacts;
        });

        feederStatic = craneLib.buildPackage {
          src = craneLib.cleanCargoSource (craneLib.path ./.);
          version = "0.1.0";
          pname = "bradipo-feeder";
          cargoExtraArgs = "-p bradipo-feeder";
          nativeBuildInputs = [
            pkgs.pkg-config
          ];

          buildInputs = with pkgs.pkgsStatic; [
            dbus
          ];

          CARGO_BUILD_TARGET = "x86_64-unknown-linux-musl";
        };

        # TODO: support building the stl file also
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = [ pkgs.pkg-config ];
          nativeBuildInputs = with pkgs; [
            cmake
            freetype
            dbus
            imagemagick
            mdbook
            rust-toolchain
            python-toolchain
            cargo-espflash
            cargo-outdated
            openscad
            taplo
            wireshark
          ];
        };

        packages = {
          inherit firmware feederStatic;
        };
      }
    );
}
