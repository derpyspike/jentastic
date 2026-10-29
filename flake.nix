{
  description = "Nix flake to compile Meshtastic firmware";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    # Shim to make flake.nix work with stable Nix.
    flake-compat = {
      url = "github:NixOS/flake-compat";
      flake = false;
    };
  };

  outputs =
    inputs:
    let
      lib = inputs.nixpkgs.lib;

      forAllSystems =
        fn:
        lib.genAttrs lib.systems.flakeExposed (
          system:
          fn {
            pkgs = import inputs.nixpkgs {
              inherit system;
            };
            inherit system;
          }
        );
    in
    {
      devShells = forAllSystems (
        { pkgs, ... }:
        let
          python3 = pkgs.python312.withPackages (
            ps: with ps; [
              google
              pip
            ]
          );

          # Mirrors .github/actions/setup-native/action.yml: libraries the
          # Portduino/native build and test suites compile and link against.
          nativeDeps = with pkgs; [
            pkg-config
            bluez
            curl
            gnutls
            i2c-tools
            jansson
            jsoncpp
            libgpiod
            libmicrohttpd
            libusb1
            libuv
            openssl
            orcania
            ulfius
            yaml-cpp
            # Native-TFT display backend (not needed by the plain native env).
            libinput
            libxkbcommon
            xorg.libX11
            yder
            zlib
          ];

          tools = with pkgs; [
            cmake
            gdb
            gnumake
            protobuf
            unzip
          ];

          # PlatformIO's native env passes bare -l flags and does not read the
          # Nix setup hooks, so expose the store paths the Debian CI gets from
          # /usr/include and /usr/lib.
          nativeOutputs = lib.concatMap (
            p: [ p ] ++ lib.optional (p ? dev) p.dev ++ lib.optional (p ? out) p.out
          ) nativeDeps;
        in
        {
          default = pkgs.mkShell {
            buildInputs = [
              python3
              pkgs.platformio
            ]
            ++ nativeDeps
            ++ tools;

            shellHook = ''
              # Native build/test dependencies (yaml-cpp, jsoncpp, libuv,
              # bluetooth, gpiod, X11, ...) live in the Nix store, not /usr.
              export CPATH="${lib.makeSearchPath "include" nativeOutputs}''${CPATH:+:$CPATH}"
              export CPLUS_INCLUDE_PATH="$CPATH"
              export LIBRARY_PATH="${lib.makeSearchPath "lib" nativeOutputs}"
              export LD_LIBRARY_PATH="${lib.makeLibraryPath nativeOutputs}''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
              export PKG_CONFIG_PATH="${lib.makeSearchPath "lib/pkgconfig" nativeOutputs}''${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
              # Set up PlatformIO to use a local core directory.
              export PLATFORMIO_CORE_DIR=$PWD/.platformio
              # Tell pip to put packages into $PIP_PREFIX instead of the usual
              # location. This is especially necessary under NixOS to avoid having
              # pip trying to write to the read-only Nix store. For more info,
              # see https://wiki.nixos.org/wiki/Python
              export PIP_PREFIX=$PWD/.python3
              export PYTHONPATH="$PIP_PREFIX/${python3.sitePackages}"
              export PATH="$PIP_PREFIX/bin:$PATH"
              # Avoids reproducibility issues with some Python packages
              # See https://nixos.org/manual/nixpkgs/stable/#python-setup.py-bdist_wheel-cannot-create-.whl
              unset SOURCE_DATE_EPOCH
            '';
          };
        }
      );
    };
}
