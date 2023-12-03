# Use cases I'd like to make easy (heavily biased by the things I like to do):
#
# - Sane default for native development (the build shell uses the stable rust minimal profile,
#   the dev shell uses the stable rust default profile, plus rust-analyzer).
#
# - Easy addition of tier 1 & 2 targets for cross-compilation.
#
# - Easy addition of extra components. Components that are added in the build shell should be
#   propagated to the dev shell also.
#
#   One fenix papercut that I'd like to avoid here is that fenix doesn't make it
#   so easy/discoverable to just add a component to an existing toolchain. Like,
#   if I'm using `stable.defaultToolchain` and I want to add rust-analyzer then
#   I either need to use `stable.withComponents` (and re-list all the components
#   that were already in the default toolchain) or use `combine`.
#
# - Use dev on nightly and build on stable (mostly useful for testing with miri, which doesn't
#   exist on stable.
#
# Known issues:
# - accidentally writing `Rust & { channel = 'nightly }` instead of `Rust & { build.channel = 'nightly }`
#   gives the dreaded "boost::bad_format_string: format-string is ill-formed" error
# - auto-completion for targets would be real nice
let inputs = import "./nickel.lock.ncl" in
let organist = inputs.organist in
let RustShell = {
  channel | [| 'stable, 'beta, 'nightly |]
          | doc "The rust channel"
          | default
          = 'stable,
  extraTargets | Array String
               | doc "Extra targets to install, for cross-compilation"
               | default
               = [],

  # There isn't a great reason to have 3 arrays here: in the end,
  # they all just get concatenated together. The point is that we want
  # good defaults and ease of adding extra components. So `buildComponents`
  # defaults to the components in rust's minimal profile, `devComponents` is
  # set in the dev shell to the components in rust's default profile, and
  # `extraComponents` is provided as a way for the user to add to those
  # components without having to restate the default values.
  #
  # All this would be much nicer if we could merge lists by
  # concatenating them...
  buildComponents | Array [| 'rustc, 'cargo, 'rustfmt, 'rust-std, 'rust-docs, 'rust-analyzer, 'clippy, 'miri, 'rust-src |]
                  | default
                  | doc m%"
                    Rust components to install in the build shell.

                    If you want to add components without removing the default ones, use `extraComponents` instead."%
                  = ['rustc, 'cargo, 'rust-std],
  devComponents | Array [| 'rustc, 'cargo, 'rustfmt, 'rust-std, 'rust-docs, 'rust-analyzer, 'clippy, 'miri, 'rust-src |]
                | default
                | doc m%"
                  Rust components to install in the dev shell.

                  If you want to add components without removing the default ones, use `extraComponents` instead."%
                = [],
  extraComponents | Array [| 'rustc, 'cargo, 'rustfmt, 'rust-std, 'rust-docs, 'rust-analyzer, 'clippy, 'miri, 'rust-src |]
                  | default
                  | doc "Extra rust components to install"
                  = [],

  packages.rust-toolchain =
    # Maybe we should default to "complete" instead of "latest"? Tier 2 targets only
    # have "latest", though...
    let toolchain = if channel == 'nightly then "latest" else std.string.from_enum channel in
    let targets = std.array.map (fun x => organist.import_nix "fenix#targets.%{x}.%{toolchain}.rust-std") extraTargets in
    organist.nix.derivation.CallNix & {
      function = "{ withComponents, components }: withComponents components",
      args = {
        withComponents = organist.import_nix "fenix#combine",
        components =
          buildComponents
          |> std.array.concat devComponents
          |> std.array.concat extraComponents
          |> std.array.map std.string.from_enum
          |> std.array.map (fun x => organist.import_nix "fenix#%{toolchain}.%{x}")
          |> std.array.concat targets
      },
    },
}
in
let Rust = {
  build = RustShell,
  dev = RustShell & {
          # This should probably have the default priority, but it gets merged with
          # build.devComponents and if it's default then they clash.
          devComponents = ['rust-docs, 'rustfmt, 'clippy, 'rust-analyzer],
        },
}
in
{
  shells = organist.shells.Bash,

  # LSP TODO: hovering on build.channel doesn't work (but autocomplete does, as long as the record is complete enough to parse)
  # LSP TODO: autocomplete for enum variants

  shells = Rust & { build.channel = 'nightly, build.extraTargets = ["riscv32imc-unknown-none-elf"], build.buildComponents = [ 'rustc, 'cargo, 'rust-std, 'rust-src ] },

  shells.build.packages.espflash = organist.import_nix "nixpkgs#cargo-espflash"
}
  | organist.OrganistExpression
