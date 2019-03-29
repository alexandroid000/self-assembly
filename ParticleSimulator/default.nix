with import <nixpkgs> {};
with pkgs.python36Packages;

let

atooms = pkgs.callPackage /home/alli/src/atooms/release.nix {
    pkgs = pkgs;
    buildPythonPackage = pkgs.python36Packages.buildPythonPackage;
};

in buildPythonPackage rec {
    version = "0.1";
    pname = "particlesim";

    buildInputs = with pkgs.python36Packages; [
        matplotlib
        scipy
        numpy
        atooms
    ];

    src = ./.;
}
