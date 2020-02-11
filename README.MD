# generate_tiles.cpp

A re-implementation of [generate_tiles.py](https://github.com/openstreetmap/mapnik-stylesheets/blob/master/generate_tiles.py) from [mapnik-stylesheets repository](https://github.com/openstreetmap/mapnik-stylesheets) to C++.

It is an almost identical rewrite:

- the code structure and variable naming are almost the same (it is one .cpp file to match the structure of the .py script)
- the algorithms and calculations are the same
- the features and configuration possibilities are almost the same

## What is different

- added configuration possibility using command-line arguments (to avoid re-compiling of the C++ code for configuration changes)
- added possibility to configure Mapnik datasources and fonts directories
- at the beginning, prints all fonts visible by Mapnik (so that it is easier to solve problems with missing fonts)
- does not need Mapnik Python bindings (obviously)

## Building

- make sure Mapnik is installed
- currently, this is only a VS Code project (no CMake or other build system)
- see .vscode/tasks.json for the dependencies and for how to build
- modify the build command-line if needed, e.g. to link against different Boost version

## Usage

- make sure you have map XML stylesheet file available (e.g. from [mapnik-stylesheets repository](https://github.com/openstreetmap/mapnik-stylesheets)) with a correctly configured and filled data source

See supported arguments:

```
./generate_tiles -h
```

Minimal usage (whole world, zooms 0 to 5):

```
 ./generate_tiles --xml ../mapnik-stylesheets/osm.xml --output ../generate_tiles_cpp_output/world
```

Choose area, zooms and threads count:

```
./generate_tiles --xml ../mapnik-stylesheets/osm.xml --output ../generate_tiles_cpp_output/sk --lat_min 47.9796 --lat_max 48.2821 --lon_min 16.9440 --lon_max 17.3831 --zoom_min 0 --zoom_max 18 --threads 4
```
