# 3dconv

![](https://github.com/brex-it/3dconv/workflows/CI/badge.svg)

Conversion tool for 3D mesh file formats. With 3dconv we can read a model from
an arbitrary mesh file (in a supported input format) make some basic
transformations and write the result into an output file in a supported output
format.

## Usage

```sh
$ 3dconv -i cube.obj -o cube.stl -f :stl-bin -T tr:1:-2.5:3.4
```

The mandatory input (`-i`) and output (`-o`) options are used for specifying
the input and output files, respectively. If they have any extensions the
program will try to determine the file formats for them, but like in the
case above when an extension is not a supported file format itself (like `stl`
because of the ambiguity of `stl-ascii` and `stl-bin` formats) we need to
specify the input and/or output formats exactly. This can be done with the `-f`
option in the form of `[input-format]:[output-format]`. If this option is used,
the format deduced from file extension is superseded by these selectors. Here
we only specified the output format, so the input format is read from the file
extension.

### Actions

The actions are order-sensitive command line options which are applied to the
model sequentially in the given order. There are several types of actions as we
can see in the following paragraphs.

With the `--transformation` or `-T` option we can apply any of the supported
affine transformations (or a combination of them) to the model. Similarly, some
face transformations are also available and can be requested by passing the
appropriate commands to the `-F` or `--face-transformation` command line
options. For the exact syntax and the full list of transformations consult the
manual (`--help` command line option).

A `--print-properties` option is also available which causes the program to
print some geometrical properties of the model.

E.g.:

```sh
$ 3dconv -i example.obj -o example.stl -f :stl-bin -p cxv -T sc:2.4 -p xvt \
-F t -p a -F c,c,t -T tr:1:4:1 -p stxv
```

A possible output of the previous command can look like this:

```txt
>>> Printing the requested properties: cxv

 * Is connected: yes
 * Is convex: no
 * Volume: 0.854563

>>> Performing model transformations: sc:2.4
>>> Printing the requested properties: xvt

 * Is convex: no
 * Is triangulated: no
 * Volume: 11.8135

>>> Performing face triangulation
>>> Printing the requested properties: a

 * Is connected: yes
 * Is convex: no
 * Surface area: 40.375
 * Is triangulated: yes
 * Volume: 11.8135
 * Is watertight: no [(Edge:1010:1016) Boundary edge]

>>> Performing face convexification
>>> Performing face triangulation
>>> Performing model transformations: tr:1:4:1
>>> Printing the requested properties: stxv

 * Is convex: no
 * Surface area: 40.375
 * Is triangulated: yes
 * Volume: 11.8056

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
