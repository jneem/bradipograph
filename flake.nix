{
  description = "Devshell for esp32c3 dev";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    fenix.url = "github:nix-community/fenix";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, fenix, flake-utils }:
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
        ];
        python-toolchain = pkgs.python3.withPackages (ps: [ps.bleak ps.matplotlib ps.numpy ps.tornado ps.pandas ps.seaborn]);
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = [ pkgs.pkg-config ];
          nativeBuildInputs = with pkgs; [
            cmake
            freetype
            dbus
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
      }
    );
}
