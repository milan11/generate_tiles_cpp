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
- possibility to specify scale, tile size and buffer size
- SVG output format
- calling additional post-processing (conversion) of the PNG or SVG files

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

The options are:

```
  -h [ --help ]                         print usage message
  --xml arg                             map XML file
  --output arg                          output directory for tiles
  --lat_min arg (=-90)                  process area from latitude
  --lat_max arg (=90)                   process area to latitude
  --lon_min arg (=-180)                 process area from longitude
  --lon_max arg (=180)                  process area to longitude
  --zoom_min arg (=0)                   min. zoom to render tiles for
  --zoom_max arg (=5)                   max. zoom to render tiles for
  --threads arg (=1)                    count of threads to use for rendering
  --tms arg (=0)                        use tms scheme (flip y)
  --datasources arg (="/usr/lib/mapnik/input")
                                        mapnik data sources directory
  --fonts arg (="/usr/share/fonts")     fonts directory
  --scale arg (=1)                      scale factor (use 2 for the 2x tiles
                                        for high-resolution displays
  --png arg (=1)                        create PNG (raster) tiles
  --svg arg (=0)                        create SVG (vector) tiles - note that
                                        SVG files for complicated tiles can be
                                        extremely large without any converting
  --convert_png arg                     conversion command to run for each PNG
                                        tile - use placeholders for file paths:
                                        $1 - absolute path of the original
                                        file, $2 - absolute path of the
                                        converted file
  --convert_svg arg                     conversion command to run for each SVG
                                        tile - use placeholders for file paths:
                                        $1 - absolute path of the original
                                        file, $2 - absolute path of the
                                        converted file
  --extension_converted_png arg (=.pngc)
                                        extension of the converted PNG file
  --extension_converted_svg arg (=.svgc)
                                        extension of the converted SVG file
  --delete_png arg (=0)                 delete original PNG file after
                                        conversion
  --delete_svg arg (=0)                 delete original SVG file after
                                        conversion
  --leave_smallest arg (=0)             leave smallest file amongst all
                                        generated files
  --tile_size arg (=256)                tile size
  --buffer_size arg (=128)              Mapnik buffer size - use higher value
                                        if some tile boundaries do not match
                                        (e.g. if a text is cut at the edge of
                                        two tiles)

```
