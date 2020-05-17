# 3dconv

![](https://github.com/brex-it/3dconv/workflows/CI/badge.svg)

Conversion tool for 3D mesh file formats. With 3dconv we can read a model from
an arbitrary mesh file (in a supported input format) make some basic
transformations and write the result into an output file in a supported output
format.

## Usage

```sh
$ 3dconv -i cube.obj -o cube.stl -t :stl-bin -T tr:1:-2.5:3.4
```

The mandatory input (`-i`) and output (`-o`) options are used for specifying
the input and output files, respectively. If they have any extensions the
program will try to determine the file formats for them, but like in the
case above when an extension is not a supported file format itself (like `stl`
because of the ambiguity of `stl-ascii` and `stl-bin` formats) we need to
specify the input and/or output formats exactly. This can be done with the `-t`
option in the form of `[input-format]:[output-format]`. If this option is used,
the format deduced from file extension is superseded by these selectors. Here
we only specified the output format, so the input format is read from the file
extension.

With the `--transformation` or `-T` option we can apply any of the supported
affine transformations (or a combination of them) to the model. For the exact
syntax and the full list of transformations consult the manual (`--help`
command line option).

A `--properties` option is also available which causes the program to print
some geometrical properties of the model and if any transformation is
performed, the same properties will be printed before and after the
transformation.

E.g.:

```txt
Model properties:
-----------------
Is connected: yes
Is convex: no
Surface area: 10.199
Is triangulated: yes
Volume: 2.19429
Is watertight: no
```

## Dependencies
* [CLI11](https://github.com/CLIUtils/CLI11) (source, included)
* [Catch2](https://github.com/catchorg/Catch2) (source, included, for unit testing)
* [Meson](https://mesonbuild.com/) (build-time)
* [ninja](https://ninja-build.org/) (build-time)
* [Doxygen](http://www.doxygen.nl/) (build-time, optional, for documentation)

## Building
The program can be built simply by issuing the following command in the
projects root directory:

```sh
$ meson . build
$ ninja -C build
```

Currently installation is not supported but the resulting binary should be
found in the build directory (`3dconv`).

## Testing
The project provides an exhaustive unittest collection. The simplest way to run
all of the tests is the following:

```sh
$ meson . build -Dbuild_tests=true
$ ninja -C build test
```

## Documentation
The code is really well documented and if doxygen is available on the build
system an html documentation will automatically be built in the
configuration phase. The default location of the html files is
`<build-dir>/doc`.

## Plugins
On how to add plugins see [Plugins](./doc/plugins.md).

## TODO
* Write more linalg unit tests
* Reconsider plugin system

## License
This software is distributed under the terms and conditions of the 3-Clause BSD
License.  
Copyright &copy; 2020, brex-it
