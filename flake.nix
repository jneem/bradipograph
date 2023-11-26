{
  inputs.nixpkgs.url = "nixpkgs/nixos-unstable";
  inputs.organist = {
    url = "github:nickel-lang/organist";
    inputs.nixpkgs.follows = "nixpkgs";
  };
  inputs.fenix = {
    url = "github:nix-community/fenix";
    inputs.nixpkgs.follows = "nixpkgs";
  };

  nixConfig = {
    extra-substituters = ["https://organist.cachix.org"];
    extra-trusted-public-keys = ["organist.cachix.org-1:GB9gOx3rbGl7YEh6DwOscD1+E/Gc5ZCnzqwObNH2Faw="];
  };

  outputs = {organist, ...} @ inputs:
    let system = "x86_64-linux"; in
    organist.flake.outputsFromNickel ./. (inputs // { fenix = inputs.fenix.packages.${system}; }) {};
}
